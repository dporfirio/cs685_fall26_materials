#!/usr/bin/env python3

"""Service-activated frontier exploration for a ROS 2 occupancy-grid map."""

import math
import threading
from collections import deque

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


class AutonomousExplorer(Node):
    """Choose information-rich frontiers and send them to Nav2 one at a time."""

    def __init__(self):
        super().__init__("autonomous_explorer")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("planning_period", 2.0)
        self.declare_parameter("free_threshold", 20)
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("minimum_frontier_cells", 8)
        self.declare_parameter("information_radius", 1.5)
        self.declare_parameter("minimum_information_gain", 0.75)
        self.declare_parameter("probe_unknown_space", True)
        self.declare_parameter("minimum_probe_information_gain", 0.20)
        self.declare_parameter("probe_spacing", 0.25)
        self.declare_parameter("minimum_progress_area", 0.35)
        self.declare_parameter("distance_penalty", 0.20)
        self.declare_parameter("obstacle_clearance", 0.25)
        self.declare_parameter("unknown_clearance", 0.10)
        self.declare_parameter("goal_search_radius", 0.75)
        self.declare_parameter("blacklist_radius", 0.50)
        self.declare_parameter("low_gain_confirmation_cycles", 3)
        self.declare_parameter("no_progress_goal_limit", 4)
        self.declare_parameter("goal_timeout", 120.0)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._map = None
        self._map_lock = threading.RLock()
        self._state_lock = threading.RLock()
        self._active = False
        self._state = "idle"
        self._detail = "Waiting for /exploration/start"
        self._goal_handle = None
        self._goal_request_pending = False
        self._goal_started_ns = None
        self._current_goal_xy = None
        self._unknown_at_goal = None
        self._replanning_after_cancel = False
        self._blacklist = []
        self._low_gain_cycles = 0
        self._no_progress_goals = 0
        self._navigation_failures = 0
        self._goals_completed = 0

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._navigator = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._map_callback,
            map_qos,
        )
        self.create_service(Trigger, "/exploration/start", self._start_callback)
        self.create_service(Trigger, "/exploration/stop", self._stop_callback)
        self.create_service(Trigger, "/exploration/status", self._status_callback)
        self._status_publisher = self.create_publisher(
            String, "/exploration/status_text", status_qos
        )
        self._goal_publisher = self.create_publisher(
            PoseStamped, "/exploration/current_goal", 1
        )

        period = float(self.get_parameter("planning_period").value)
        self.create_timer(period, self._planning_tick)
        self._publish_status()

    def _map_callback(self, message):
        with self._map_lock:
            self._map = message

    def _start_callback(self, _request, response):
        with self._state_lock:
            if self._active:
                response.success = False
                response.message = "Exploration is already active"
                return response
            if self._goal_request_pending or self._goal_handle is not None:
                response.success = False
                response.message = "Waiting for the previous goal to cancel"
                return response
            if self._map is None:
                response.success = False
                response.message = "No occupancy grid has been received"
                return response
            if not self._navigator.server_is_ready():
                response.success = False
                response.message = "Nav2 /navigate_to_pose is not available"
                return response

            self._active = True
            self._blacklist.clear()
            self._low_gain_cycles = 0
            self._no_progress_goals = 0
            self._navigation_failures = 0
            self._goals_completed = 0
            self._set_status("planning", "Exploration activated")
            response.success = True
            response.message = "Exploration started"
            return response

    def _stop_callback(self, _request, response):
        with self._state_lock:
            was_active = self._active
            self._active = False
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
            self._set_status("stopped", "Stopped by service request")
            response.success = was_active
            response.message = (
                "Exploration stopped" if was_active else "Exploration was not active"
            )
            return response

    def _status_callback(self, _request, response):
        with self._state_lock:
            response.success = self._active
            response.message = self._status_message()
            return response

    def _planning_tick(self):
        with self._state_lock:
            if not self._active:
                return
            if self._goal_handle is not None or self._goal_request_pending:
                if self._goal_handle is not None:
                    self._revalidate_current_goal()
                self._check_goal_timeout()
                return

        map_snapshot = self._copy_map()
        if map_snapshot is None:
            self._set_status("waiting_for_map", "Waiting for an occupancy grid")
            return

        robot_position = self._robot_position()
        if robot_position is None:
            return

        candidate = self._choose_frontier(map_snapshot, robot_position)
        if candidate is None:
            self._low_gain_cycles += 1
            required = int(
                self.get_parameter("low_gain_confirmation_cycles").value
            )
            self._set_status(
                "checking_completion",
                f"No sufficiently informative frontier "
                f"({self._low_gain_cycles}/{required})",
            )
            if self._low_gain_cycles >= required:
                self._finish(
                    "No frontier with enough estimated information gain remains"
                )
            return

        self._low_gain_cycles = 0
        x, y, yaw, gain, frontier_size = candidate
        pose = self._make_goal(x, y, yaw, map_snapshot.header.frame_id)
        self._send_goal(pose, gain, frontier_size, map_snapshot)

    def _copy_map(self):
        with self._map_lock:
            if self._map is None:
                return None
            return self._map

    def _robot_position(self):
        map_frame = str(self.get_parameter("map_frame").value)
        odom_frame = str(self.get_parameter("odom_frame").value)
        robot_frame = str(self.get_parameter("robot_frame").value)
        try:
            transform = self._tf_buffer.lookup_transform(
                map_frame, robot_frame, rclpy.time.Time()
            )
        except TransformException as error:
            # MuJoCo and slam_toolbox can publish the two dynamic legs of this
            # chain with non-overlapping timestamp histories. A normal chained
            # lookup then reports extrapolation even though both latest
            # transforms are available. Compose their latest 2-D poses; this is
            # sufficient for selecting frontier goals on the occupancy grid.
            try:
                map_to_odom = self._tf_buffer.lookup_transform(
                    map_frame, odom_frame, rclpy.time.Time()
                )
                odom_to_robot = self._tf_buffer.lookup_transform(
                    odom_frame, robot_frame, rclpy.time.Time()
                )
                return self._compose_planar_translation(
                    map_to_odom, odom_to_robot
                )
            except TransformException as fallback_error:
                self._set_status(
                    "waiting_for_tf",
                    f"Waiting for robot pose: {error}; "
                    f"separate-chain lookup also failed: {fallback_error}",
                )
                return None
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    @staticmethod
    def _compose_planar_translation(parent_to_middle, middle_to_child):
        first = parent_to_middle.transform
        second = middle_to_child.transform
        quaternion = first.rotation
        yaw = math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
        )
        return (
            first.translation.x
            + math.cos(yaw) * second.translation.x
            - math.sin(yaw) * second.translation.y,
            first.translation.y
            + math.sin(yaw) * second.translation.x
            + math.cos(yaw) * second.translation.y,
        )

    def _choose_frontier(self, map_message, robot_position):
        width = map_message.info.width
        height = map_message.info.height
        if width == 0 or height == 0:
            return None

        grid = np.asarray(map_message.data, dtype=np.int16).reshape((height, width))
        free_threshold = int(self.get_parameter("free_threshold").value)
        occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        free = (grid >= 0) & (grid <= free_threshold)
        unknown = grid < 0
        occupied = grid >= occupied_threshold

        robot_cell = self._world_to_grid(
            map_message, robot_position[0], robot_position[1]
        )

        adjacent_unknown = np.zeros_like(unknown)
        adjacent_unknown[1:, :] |= unknown[:-1, :]
        adjacent_unknown[:-1, :] |= unknown[1:, :]
        adjacent_unknown[:, 1:] |= unknown[:, :-1]
        adjacent_unknown[:, :-1] |= unknown[:, 1:]
        frontier_mask = free & adjacent_unknown

        clusters = self._frontier_clusters(frontier_mask)
        minimum_cells = int(self.get_parameter("minimum_frontier_cells").value)
        resolution = map_message.info.resolution
        minimum_gain = float(
            self.get_parameter("minimum_information_gain").value
        )
        information_cells = max(
            1,
            int(
                round(
                    float(self.get_parameter("information_radius").value)
                    / resolution
                )
            ),
        )
        clearance_cells = max(
            0,
            int(
                math.ceil(
                    float(self.get_parameter("obstacle_clearance").value)
                    / resolution
                )
            ),
        )
        unknown_clearance_cells = max(
            0,
            int(
                math.ceil(
                    float(self.get_parameter("unknown_clearance").value)
                    / resolution
                )
            ),
        )
        search_cells = max(
            1,
            int(
                math.ceil(
                    float(self.get_parameter("goal_search_radius").value)
                    / resolution
                )
            ),
        )
        traversable = self._clearance_constrained_free(
            free,
            occupied,
            unknown,
            clearance_cells,
            unknown_clearance_cells,
        )
        reachable = self._reachable_free(traversable, robot_cell)
        distance_penalty = float(self.get_parameter("distance_penalty").value)

        best = None
        for cluster in clusters:
            if len(cluster) < minimum_cells:
                continue
            row, col = self._cluster_goal_cell(
                cluster,
                free,
                occupied,
                unknown,
                reachable,
                clearance_cells,
                unknown_clearance_cells,
                search_cells,
            )
            if row is None:
                continue
            x, y = self._grid_to_world(map_message, row, col)
            if self._is_blacklisted(x, y):
                continue

            gain_cells = self._unknown_in_radius(
                unknown, row, col, information_cells
            )
            gain = gain_cells * resolution * resolution
            if gain < minimum_gain:
                continue
            distance = math.hypot(x - robot_position[0], y - robot_position[1])
            score = gain - distance_penalty * distance
            yaw = self._frontier_yaw(map_message, unknown, row, col)
            if best is None or score > best[0]:
                best = (score, x, y, yaw, gain, len(cluster))

        if best is None and bool(self.get_parameter("probe_unknown_space").value):
            return self._choose_unknown_space_probe(
                map_message,
                unknown,
                reachable,
                robot_position,
                information_cells,
                distance_penalty,
            )
        if best is None:
            return None
        _, x, y, yaw, gain, frontier_size = best
        return x, y, yaw, gain, frontier_size

    def _choose_unknown_space_probe(
        self,
        map_message,
        unknown,
        reachable,
        robot_position,
        information_cells,
        distance_penalty,
    ):
        """Find a safe viewpoint for unknown space missed by frontier filtering."""
        resolution = map_message.info.resolution
        spacing_cells = max(
            1,
            int(
                round(
                    float(self.get_parameter("probe_spacing").value) / resolution
                )
            ),
        )
        minimum_gain = float(
            self.get_parameter("minimum_probe_information_gain").value
        )
        candidates = np.argwhere(reachable)
        best = None
        for row, col in candidates:
            if row % spacing_cells != 0 or col % spacing_cells != 0:
                continue
            gain_cells = self._unknown_in_radius(
                unknown, int(row), int(col), information_cells
            )
            gain = gain_cells * resolution * resolution
            if gain < minimum_gain:
                continue
            x, y = self._grid_to_world(map_message, int(row), int(col))
            if self._is_blacklisted(x, y):
                continue
            distance = math.hypot(x - robot_position[0], y - robot_position[1])
            score = gain - distance_penalty * distance
            yaw = self._frontier_yaw(
                map_message, unknown, int(row), int(col)
            )
            if best is None or score > best[0]:
                best = (score, x, y, yaw, gain)

        if best is None:
            return None
        _, x, y, yaw, gain = best
        return x, y, yaw, gain, 0

    @staticmethod
    def _frontier_clusters(mask):
        visited = np.zeros_like(mask, dtype=bool)
        clusters = []
        height, width = mask.shape
        for start_row, start_col in np.argwhere(mask):
            if visited[start_row, start_col]:
                continue
            queue = deque([(int(start_row), int(start_col))])
            visited[start_row, start_col] = True
            cluster = []
            while queue:
                row, col = queue.popleft()
                cluster.append((row, col))
                for row_step in (-1, 0, 1):
                    for col_step in (-1, 0, 1):
                        if row_step == 0 and col_step == 0:
                            continue
                        next_row = row + row_step
                        next_col = col + col_step
                        if not (0 <= next_row < height and 0 <= next_col < width):
                            continue
                        if mask[next_row, next_col] and not visited[next_row, next_col]:
                            visited[next_row, next_col] = True
                            queue.append((next_row, next_col))
            clusters.append(cluster)
        return clusters

    @classmethod
    def _cluster_goal_cell(
        cls,
        cluster,
        free,
        occupied,
        unknown,
        reachable,
        obstacle_clearance,
        unknown_clearance,
        search_radius,
    ):
        points = np.asarray(cluster, dtype=np.int32)
        center = points.mean(axis=0)
        height, width = free.shape
        row_min = max(0, int(points[:, 0].min()) - search_radius)
        row_max = min(height, int(points[:, 0].max()) + search_radius + 1)
        col_min = max(0, int(points[:, 1].min()) - search_radius)
        col_max = min(width, int(points[:, 1].max()) + search_radius + 1)

        candidates = np.argwhere(
            free[row_min:row_max, col_min:col_max]
            & reachable[row_min:row_max, col_min:col_max]
        )
        if candidates.size == 0:
            return None, None
        candidates += np.array([row_min, col_min])
        order = np.argsort(np.sum((candidates - center) ** 2, axis=1))
        for index in order:
            row, col = candidates[index]
            if cls._cell_has_clearance(
                occupied, int(row), int(col), obstacle_clearance
            ) and cls._cell_has_clearance(
                unknown, int(row), int(col), unknown_clearance
            ):
                return int(row), int(col)
        return None, None

    @staticmethod
    def _cell_has_clearance(mask, row, col, clearance):
        height, width = mask.shape
        row_min = max(0, row - clearance)
        row_max = min(height, row + clearance + 1)
        col_min = max(0, col - clearance)
        col_max = min(width, col + clearance + 1)
        return not mask[row_min:row_max, col_min:col_max].any()

    @staticmethod
    def _clearance_constrained_free(
        free, occupied, unknown, obstacle_clearance, unknown_clearance
    ):
        safe = free.copy()
        for mask, clearance in (
            (occupied, obstacle_clearance),
            (unknown, unknown_clearance),
        ):
            if clearance <= 0:
                safe &= ~mask
                continue
            padded = np.pad(mask, clearance, mode="constant", constant_values=True)
            blocked = np.zeros_like(mask, dtype=bool)
            size = 2 * clearance + 1
            for row_offset in range(size):
                for col_offset in range(size):
                    blocked |= padded[
                        row_offset : row_offset + mask.shape[0],
                        col_offset : col_offset + mask.shape[1],
                    ]
            safe &= ~blocked
        return safe

    @staticmethod
    def _reachable_free(free, start):
        reachable = np.zeros_like(free, dtype=bool)
        if start is None:
            return reachable
        start_row, start_col = start
        height, width = free.shape
        if not (0 <= start_row < height and 0 <= start_col < width):
            return reachable
        if not free[start_row, start_col]:
            nearby = np.argwhere(free)
            if nearby.size == 0:
                return reachable
            nearest = nearby[
                np.argmin(np.sum((nearby - np.array(start)) ** 2, axis=1))
            ]
            start_row, start_col = int(nearest[0]), int(nearest[1])
        queue = deque([(start_row, start_col)])
        reachable[start_row, start_col] = True
        while queue:
            row, col = queue.popleft()
            for next_row, next_col in (
                (row - 1, col),
                (row + 1, col),
                (row, col - 1),
                (row, col + 1),
            ):
                if (
                    0 <= next_row < height
                    and 0 <= next_col < width
                    and free[next_row, next_col]
                    and not reachable[next_row, next_col]
                ):
                    reachable[next_row, next_col] = True
                    queue.append((next_row, next_col))
        return reachable

    @staticmethod
    def _unknown_in_radius(unknown, row, col, radius):
        height, width = unknown.shape
        row_min = max(0, row - radius)
        row_max = min(height, row + radius + 1)
        col_min = max(0, col - radius)
        col_max = min(width, col + radius + 1)
        local = unknown[row_min:row_max, col_min:col_max]
        rows, cols = np.ogrid[row_min:row_max, col_min:col_max]
        circle = (rows - row) ** 2 + (cols - col) ** 2 <= radius**2
        return int(np.count_nonzero(local & circle))

    def _frontier_yaw(self, map_message, unknown, row, col):
        radius = 2
        row_min = max(0, row - radius)
        row_max = min(unknown.shape[0], row + radius + 1)
        col_min = max(0, col - radius)
        col_max = min(unknown.shape[1], col + radius + 1)
        unknown_points = np.argwhere(unknown[row_min:row_max, col_min:col_max])
        if unknown_points.size == 0:
            return 0.0
        target_row, target_col = unknown_points.mean(axis=0)
        target_row += row_min
        target_col += col_min
        x, y = self._grid_to_world(map_message, row, col)
        target_x, target_y = self._grid_to_world(
            map_message, target_row, target_col
        )
        return math.atan2(target_y - y, target_x - x)

    @staticmethod
    def _grid_to_world(map_message, row, col):
        origin = map_message.info.origin
        resolution = map_message.info.resolution
        quaternion = origin.orientation
        yaw = math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
        )
        local_x = (float(col) + 0.5) * resolution
        local_y = (float(row) + 0.5) * resolution
        return (
            origin.position.x + math.cos(yaw) * local_x - math.sin(yaw) * local_y,
            origin.position.y + math.sin(yaw) * local_x + math.cos(yaw) * local_y,
        )

    @staticmethod
    def _world_to_grid(map_message, x, y):
        origin = map_message.info.origin
        quaternion = origin.orientation
        yaw = math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
        )
        delta_x = x - origin.position.x
        delta_y = y - origin.position.y
        local_x = math.cos(yaw) * delta_x + math.sin(yaw) * delta_y
        local_y = -math.sin(yaw) * delta_x + math.cos(yaw) * delta_y
        resolution = map_message.info.resolution
        return (
            int(math.floor(local_y / resolution)),
            int(math.floor(local_x / resolution)),
        )

    def _is_blacklisted(self, x, y):
        radius = float(self.get_parameter("blacklist_radius").value)
        return any(math.hypot(x - bx, y - by) <= radius for bx, by in self._blacklist)

    def _make_goal(self, x, y, yaw, frame_id):
        pose = PoseStamped()
        pose.header.frame_id = frame_id or str(self.get_parameter("map_frame").value)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _send_goal(self, pose, gain, frontier_size, map_message):
        goal = NavigateToPose.Goal()
        goal.pose = pose
        with self._state_lock:
            self._goal_request_pending = True
            self._current_goal_xy = (
                pose.pose.position.x,
                pose.pose.position.y,
            )
            self._unknown_at_goal = sum(value < 0 for value in map_message.data)
            goal_source = (
                f"{frontier_size} frontier cells"
                if frontier_size > 0
                else "a remaining unknown-space viewpoint"
            )
            self._set_status(
                "navigating",
                f"Goal ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}); "
                f"estimated gain {gain:.2f} m^2 from {goal_source}",
            )
        self._goal_publisher.publish(pose)
        future = self._navigator.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        with self._state_lock:
            self._goal_request_pending = False
            try:
                goal_handle = future.result()
            except Exception as error:  # rclpy futures surface transport errors here
                self._record_goal_failure(f"Goal request failed: {error}")
                return
            if not goal_handle.accepted:
                self._record_goal_failure("Nav2 rejected the frontier goal")
                return
            if not self._active:
                goal_handle.cancel_goal_async()
                self._current_goal_xy = None
                return
            self._goal_handle = goal_handle
            self._goal_started_ns = self.get_clock().now().nanoseconds
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        with self._state_lock:
            self._goal_handle = None
            self._goal_started_ns = None
            if not self._active:
                return
            if self._replanning_after_cancel:
                self._replanning_after_cancel = False
                self._current_goal_xy = None
                self._set_status(
                    "planning", "Canceled stale goal; selecting a new frontier"
                )
                return
            try:
                wrapped_result = future.result()
                status = wrapped_result.status
            except Exception as error:
                self._record_goal_failure(f"Navigation result failed: {error}")
                return

            if status != GoalStatus.STATUS_SUCCEEDED:
                self._record_goal_failure(f"Navigation ended with status {status}")
                return

            self._current_goal_xy = None
            self._goals_completed += 1
            current_unknown = self._current_unknown_count()
            learned_area = 0.0
            if current_unknown is not None and self._unknown_at_goal is not None:
                resolution = self._map.info.resolution
                learned_area = max(0, self._unknown_at_goal - current_unknown) * resolution**2
            minimum_progress = float(
                self.get_parameter("minimum_progress_area").value
            )
            if learned_area < minimum_progress:
                self._no_progress_goals += 1
            else:
                self._no_progress_goals = 0
            self._navigation_failures = 0

            limit = int(self.get_parameter("no_progress_goal_limit").value)
            self._set_status(
                "planning",
                f"Goal succeeded; learned approximately {learned_area:.2f} m^2",
            )
            if self._no_progress_goals >= limit:
                self._finish(
                    f"The last {limit} goals produced less than "
                    f"{minimum_progress:.2f} m^2 of new map each"
                )

    def _record_goal_failure(self, detail):
        if self._current_goal_xy is not None:
            self._blacklist.append(self._current_goal_xy)
            self._current_goal_xy = None
        # Rejections and aborted paths are expected as SLAM reveals obstacles.
        # Blacklist this location and keep looking; only measured low information
        # gain or exhaustion of useful candidates should complete exploration.
        self._navigation_failures += 1
        self._set_status(
            "planning",
            f"{detail}; blacklisted that goal and selecting another "
            f"(navigation failures: {self._navigation_failures})",
        )

    def _revalidate_current_goal(self):
        if self._current_goal_xy is None or self._replanning_after_cancel:
            return
        map_message = self._copy_map()
        robot_position = self._robot_position()
        if map_message is None or robot_position is None:
            return

        width = map_message.info.width
        height = map_message.info.height
        grid = np.asarray(map_message.data, dtype=np.int16).reshape((height, width))
        free_threshold = int(self.get_parameter("free_threshold").value)
        occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        free = (grid >= 0) & (grid <= free_threshold)
        occupied = grid >= occupied_threshold
        unknown = grid < 0
        goal_row, goal_col = self._world_to_grid(
            map_message, self._current_goal_xy[0], self._current_goal_xy[1]
        )
        robot_cell = self._world_to_grid(
            map_message, robot_position[0], robot_position[1]
        )
        resolution = map_message.info.resolution
        obstacle_clearance = max(
            0,
            int(
                math.ceil(
                    float(self.get_parameter("obstacle_clearance").value)
                    / resolution
                )
            ),
        )
        unknown_clearance = max(
            0,
            int(
                math.ceil(
                    float(self.get_parameter("unknown_clearance").value)
                    / resolution
                )
            ),
        )
        in_bounds = 0 <= goal_row < height and 0 <= goal_col < width
        traversable = self._clearance_constrained_free(
            free,
            occupied,
            unknown,
            obstacle_clearance,
            unknown_clearance,
        )
        reachable = self._reachable_free(traversable, robot_cell)
        feasible = (
            in_bounds
            and free[goal_row, goal_col]
            and reachable[goal_row, goal_col]
            and self._cell_has_clearance(
                occupied, goal_row, goal_col, obstacle_clearance
            )
            and self._cell_has_clearance(
                unknown, goal_row, goal_col, unknown_clearance
            )
        )
        if feasible:
            return

        self._blacklist.append(self._current_goal_xy)
        self._replanning_after_cancel = True
        self._set_status(
            "canceling_goal",
            "The map changed and the current goal is no longer safely reachable",
        )
        self._goal_handle.cancel_goal_async()

    def _check_goal_timeout(self):
        if (
            self._goal_handle is None
            or self._goal_started_ns is None
            or self._replanning_after_cancel
        ):
            return
        elapsed = (self.get_clock().now().nanoseconds - self._goal_started_ns) / 1e9
        timeout = float(self.get_parameter("goal_timeout").value)
        if elapsed > timeout:
            self._goal_handle.cancel_goal_async()
            self._goal_started_ns = None
            self._set_status(
                "canceling_goal", f"Frontier goal timed out after {timeout:.0f} s"
            )

    def _current_unknown_count(self):
        with self._map_lock:
            if self._map is None:
                return None
            return sum(value < 0 for value in self._map.data)

    def _finish(self, reason):
        self._active = False
        self._set_status("complete", reason)
        self.get_logger().info(f"Exploration complete: {reason}")

    def _set_status(self, state, detail):
        self._state = state
        self._detail = detail
        self._publish_status()

    def _status_message(self):
        return (
            f"state={self._state}; active={self._active}; "
            f"goals_completed={self._goals_completed}; "
            f"navigation_failures={self._navigation_failures}; {self._detail}"
        )

    def _publish_status(self):
        if not hasattr(self, "_status_publisher"):
            return
        message = String()
        message.data = self._status_message()
        self._status_publisher.publish(message)
        self.get_logger().info(message.data)


def main():
    rclpy.init()
    node = AutonomousExplorer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
