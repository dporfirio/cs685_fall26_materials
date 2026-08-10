#!/usr/bin/env python3

"""Execute named kitchen-item navigation goals in the requested order."""

import math
from collections import deque

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


# Values are ((goto_x, goto_y), (point_at_x, point_at_y)).
ITEM_LOCATIONS = {
    "spoon": ((1.12, -1.24), (0.67, -0.29)),
    "pan": ((1.44, -1.24), (1.24, -0.29)),
    "patty": ((1.71, -1.24), (1.90, -0.29)),
    "chicken": ((2.40, -1.24), (2.48, -0.29)),
    "tomato": ((3.87, -1.24), (3.76, -0.29)),
    "lettuce": ((4.20, -1.24), (4.17, -0.29)),
    "bacon": ((4.64, -1.24), (4.90, -0.29)),
    "bread": ((4.67, -1.49), (5.54, -1.22)),
    "knife": ((4.67, -1.49), (5.50, -1.88)),
    "plate": ((3.94, -1.99), (4.25, -2.88)),
    "avocado": ((1.30, -2.11), (1.09, -3.05)),
}


def parse_item_sequence(text):
    """Return normalized item names, rejecting empty or unknown entries."""
    items = [item.strip().lower() for item in text.split(",")]
    if not items or any(not item for item in items):
        raise ValueError("provide a comma-separated list with no empty items")
    unknown = sorted(set(items) - ITEM_LOCATIONS.keys())
    if unknown:
        raise ValueError(f"unknown item(s): {', '.join(unknown)}")
    return items


def navigation_yaw(item, gripper_bearing_offset):
    """Compute base yaw so the gripper's bearing intersects point_at."""
    (goto_x, goto_y), (point_x, point_y) = ITEM_LOCATIONS[item]
    target_bearing = math.atan2(point_y - goto_y, point_x - goto_x)
    return target_bearing - gripper_bearing_offset


class KitchenItemTraveler(Node):
    """Turn comma-separated item names into sequential Nav2 goals."""

    def __init__(self):
        super().__init__("kitchen_item_traveler")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("robot_frame", "base_link")
        # This Stretch model's arm/gripper extends to the right (-pi/2) from
        # base_link, so its right flank must face the Point-at coordinate.
        self.declare_parameter("gripper_bearing_offset", -math.pi / 2.0)
        self.declare_parameter("maximum_lift_height", 1.1)
        self.declare_parameter("pointing_lift_fraction", 0.8)
        self.declare_parameter("maximum_arm_extension", 0.52)
        self.declare_parameter("pointing_extension_fraction", 0.33)
        self.declare_parameter("pointing_wrist_yaw", 0.0)
        self.declare_parameter("pointing_wrist_pitch", 0.0)
        self.declare_parameter("travel_lift_height", 0.3)
        self.declare_parameter("travel_wrist_yaw", 2.8)
        self.declare_parameter("travel_wrist_pitch", -0.4)
        self.declare_parameter("arm_motion_duration", 2.0)
        self.declare_parameter("pointing_hold_duration", 1.5)
        self.declare_parameter("turn_tolerance", 0.08)
        self.declare_parameter("turn_kp", 0.8)
        self.declare_parameter("turn_minimum_speed", 0.08)
        self.declare_parameter("turn_maximum_speed", 0.35)
        self.declare_parameter("turn_timeout", 60.0)

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._navigator = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._arm_controller = ActionClient(
            self,
            FollowJointTrajectory,
            "/stretch_controller/follow_joint_trajectory",
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._queue = deque()
        self._goal_handle = None
        self._goal_request_pending = False
        self._turn_needed = False
        self._turn_started_ns = None
        self._last_turn_status_ns = None
        self._pointing_yaw = None
        self._arm_goal_handle = None
        self._arm_request_pending = False
        self._hold_timer = None
        self._active_item = None
        self._completed = 0
        self._total = 0
        self._stopping = False
        self._state = "idle"
        self._detail = "Waiting for /kitchen_travel/items"

        self.create_subscription(
            String, "/kitchen_travel/items", self._items_callback, 10
        )
        self._status_publisher = self.create_publisher(
            String, "/kitchen_travel/status_text", status_qos
        )
        self._velocity_publisher = self.create_publisher(
            Twist, "/stretch/cmd_vel", 10
        )
        self.create_service(Trigger, "/kitchen_travel/stop", self._stop_callback)
        self.create_service(Trigger, "/kitchen_travel/status", self._status_callback)
        self.create_timer(0.25, self._dispatch_next)
        self.create_timer(0.05, self._turn_tick)
        self._publish_status()

    def _items_callback(self, message):
        if self._is_active():
            self.get_logger().warning("Ignored item list: a sequence is already active")
            return
        try:
            items = parse_item_sequence(message.data)
        except ValueError as error:
            self._set_status("rejected", str(error))
            self.get_logger().error(f"Rejected item list: {error}")
            return

        self._queue.extend(items)
        self._stopping = False
        self._completed = 0
        self._total = len(items)
        self._set_status("queued", f"Accepted {self._total} item(s)")

    def _dispatch_next(self):
        if (
            self._active_item is not None
            or self._goal_handle is not None
            or self._goal_request_pending
            or self._turn_needed
            or self._arm_goal_handle is not None
            or self._arm_request_pending
            or self._hold_timer is not None
        ):
            return
        if not self._queue:
            return
        if not self._navigator.server_is_ready():
            self._set_status("waiting_for_nav2", "Waiting for /navigate_to_pose")
            return
        if not self._arm_controller.server_is_ready():
            self._set_status(
                "waiting_for_arm", "Waiting for the Stretch trajectory controller"
            )
            return
        robot_pose = self._robot_pose()
        if robot_pose is None:
            return

        item = self._queue.popleft()
        (goto_x, goto_y), _ = ITEM_LOCATIONS[item]
        offset = float(self.get_parameter("gripper_bearing_offset").value)
        self._pointing_yaw = navigation_yaw(item, offset)
        # Finish the driving goal facing along the approach direction. Asking
        # DWB to follow the path while also settling a perpendicular gripper
        # yaw can make its path-alignment and goal-alignment critics oscillate.
        yaw = math.atan2(goto_y - robot_pose[1], goto_x - robot_pose[0])

        pose = PoseStamped()
        pose.header.frame_id = str(self.get_parameter("map_frame").value)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = goto_x
        pose.pose.position.y = goto_y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._active_item = item
        self._goal_request_pending = True
        self._set_status(
            "navigating",
            f"{self._completed + 1}/{self._total}: {item} -> "
            f"({goto_x:.2f}, {goto_y:.2f})",
        )
        future = self._navigator.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        self._goal_request_pending = False
        try:
            goal_handle = future.result()
        except Exception as error:
            if self._stopping:
                return
            self._abort(f"Goal request for {self._active_item} failed: {error}")
            return
        if not goal_handle.accepted:
            if self._stopping:
                return
            self._abort(f"Nav2 rejected the goal for {self._active_item}")
            return
        self._goal_handle = goal_handle
        if self._stopping:
            goal_handle.cancel_goal_async()
        goal_handle.get_result_async().add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        item = self._active_item
        self._goal_handle = None
        if self._stopping:
            self._goal_request_pending = False
            return
        try:
            status = future.result().status
        except Exception as error:
            self._abort(f"Navigation result for {item} failed: {error}")
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._abort(f"Navigation to {item} ended with status {status}")
            return

        self._turn_needed = True
        self._turn_started_ns = self.get_clock().now().nanoseconds
        self._last_turn_status_ns = None
        self._set_status("waiting_to_turn", f"Preparing to turn toward {item}")

    def _turn_tick(self):
        if not self._turn_needed or self._stopping:
            return
        robot_pose = self._robot_pose()
        if robot_pose is None:
            return
        yaw_error = self._normalize_angle(self._pointing_yaw - robot_pose[2])
        tolerance = max(0.01, float(self.get_parameter("turn_tolerance").value))
        if abs(yaw_error) <= tolerance:
            self._publish_turn_stop()
            self._turn_needed = False
            self._turn_started_ns = None
            self._begin_pointing(self._active_item)
            return

        elapsed = (
            self.get_clock().now().nanoseconds - self._turn_started_ns
        ) / 1e9
        timeout = max(1.0, float(self.get_parameter("turn_timeout").value))
        if elapsed > timeout:
            self._publish_turn_stop()
            self._abort(f"Base turn timed out with {yaw_error:.2f} rad remaining")
            return

        kp = max(0.0, float(self.get_parameter("turn_kp").value))
        minimum = max(
            0.0, float(self.get_parameter("turn_minimum_speed").value)
        )
        maximum = max(
            minimum, float(self.get_parameter("turn_maximum_speed").value)
        )
        speed = min(maximum, max(minimum, kp * abs(yaw_error)))
        command = Twist()
        command.angular.z = math.copysign(speed, yaw_error)
        self._velocity_publisher.publish(command)
        now_ns = self.get_clock().now().nanoseconds
        if (
            self._last_turn_status_ns is None
            or (now_ns - self._last_turn_status_ns) / 1e9 >= 0.5
        ):
            self._last_turn_status_ns = now_ns
            self._set_status(
                "turning",
                f"Turning toward {self._active_item}; "
                f"yaw error {yaw_error:.2f} rad",
            )

    def _publish_turn_stop(self):
        self._velocity_publisher.publish(Twist())

    def _robot_pose(self):
        map_frame = str(self.get_parameter("map_frame").value)
        odom_frame = str(self.get_parameter("odom_frame").value)
        robot_frame = str(self.get_parameter("robot_frame").value)
        try:
            transform = self._tf_buffer.lookup_transform(
                map_frame,
                robot_frame,
                rclpy.time.Time(),
            )
        except TransformException as error:
            # MuJoCo and slam_toolbox can have non-overlapping timestamp
            # histories. Compose their latest map->odom and odom->base poses.
            try:
                map_to_odom = self._tf_buffer.lookup_transform(
                    map_frame, odom_frame, rclpy.time.Time()
                )
                odom_to_robot = self._tf_buffer.lookup_transform(
                    odom_frame, robot_frame, rclpy.time.Time()
                )
            except TransformException as fallback_error:
                self._set_status(
                    "waiting_for_tf",
                    f"Waiting for robot pose: {error}; {fallback_error}",
                )
                return None
            first = self._planar_pose(map_to_odom)
            second = self._planar_pose(odom_to_robot)
            return (
                first[0]
                + math.cos(first[2]) * second[0]
                - math.sin(first[2]) * second[1],
                first[1]
                + math.sin(first[2]) * second[0]
                + math.cos(first[2]) * second[1],
                self._normalize_angle(first[2] + second[2]),
            )
        return self._planar_pose(transform)

    @staticmethod
    def _planar_pose(transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y**2 + rotation.z**2),
        )
        return translation.x, translation.y, yaw

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _begin_pointing(self, item):
        lift = self._fractional_target(
            "maximum_lift_height", "pointing_lift_fraction"
        )
        self._set_status(
            "raising_arm",
            f"Raising the retracted arm to {lift:.2f} m for {item}",
        )
        self._send_arm_trajectory(
            "raise",
            ["joint_lift", "wrist_extension"],
            [lift, 0.0],
            self._aim_gripper,
        )

    def _aim_gripper(self):
        self._set_status(
            "aiming", f"Aiming the retracted gripper toward {self._active_item}"
        )
        self._send_arm_trajectory(
            "aim",
            ["joint_wrist_pitch", "joint_wrist_roll", "joint_wrist_yaw"],
            [
                float(self.get_parameter("pointing_wrist_pitch").value),
                0.0,
                float(self.get_parameter("pointing_wrist_yaw").value),
            ],
            self._extend_arm,
        )

    def _extend_arm(self):
        extension = self._fractional_target(
            "maximum_arm_extension", "pointing_extension_fraction"
        )
        self._set_status(
            "pointing",
            f"Extending the gripper {extension:.2f} m toward {self._active_item}",
        )
        self._send_arm_trajectory(
            "extend",
            ["wrist_extension"],
            [extension],
            self._hold_point,
        )

    def _hold_point(self):
        duration = max(
            0.0, float(self.get_parameter("pointing_hold_duration").value)
        )
        self._set_status(
            "pointing",
            f"Pointing at {self._active_item} for {duration:.1f} s",
        )
        if duration == 0.0:
            self._retract_arm()
            return
        self._hold_timer = self.create_timer(duration, self._hold_finished)

    def _hold_finished(self):
        if self._hold_timer is not None:
            self._hold_timer.cancel()
            self.destroy_timer(self._hold_timer)
            self._hold_timer = None
        if not self._stopping:
            self._retract_arm()

    def _retract_arm(self):
        self._set_status(
            "retracting", f"Retracting the arm from {self._active_item}"
        )
        self._send_arm_trajectory(
            "retract",
            ["wrist_extension"],
            [0.0],
            self._restore_travel_posture,
        )

    def _restore_travel_posture(self):
        self._set_status(
            "stowing", f"Restoring the travel posture after {self._active_item}"
        )
        self._send_arm_trajectory(
            "restore travel posture",
            [
                "joint_lift",
                "wrist_extension",
                "joint_wrist_pitch",
                "joint_wrist_roll",
                "joint_wrist_yaw",
            ],
            [
                float(self.get_parameter("travel_lift_height").value),
                0.0,
                float(self.get_parameter("travel_wrist_pitch").value),
                0.0,
                float(self.get_parameter("travel_wrist_yaw").value),
            ],
            self._item_complete,
        )

    def _send_arm_trajectory(self, stage, joint_names, positions, next_callback):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        duration = max(
            0.1, float(self.get_parameter("arm_motion_duration").value)
        )
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        goal.trajectory.points.append(point)
        self._arm_request_pending = True
        future = self._arm_controller.send_goal_async(goal)
        future.add_done_callback(
            lambda result: self._arm_response_callback(
                result, stage, next_callback
            )
        )

    def _arm_response_callback(self, future, stage, next_callback):
        self._arm_request_pending = False
        try:
            goal_handle = future.result()
        except Exception as error:
            if not self._stopping:
                self._abort(f"Arm {stage} request failed: {error}")
            return
        if not goal_handle.accepted:
            if not self._stopping:
                self._abort(f"The trajectory controller rejected arm {stage}")
            return
        self._arm_goal_handle = goal_handle
        if self._stopping:
            goal_handle.cancel_goal_async()
        goal_handle.get_result_async().add_done_callback(
            lambda result: self._arm_result_callback(
                result, stage, next_callback
            )
        )

    def _arm_result_callback(self, future, stage, next_callback):
        self._arm_goal_handle = None
        if self._stopping:
            return
        try:
            status = future.result().status
        except Exception as error:
            self._abort(f"Arm {stage} result failed: {error}")
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._abort(f"Arm {stage} ended with status {status}")
            return
        next_callback()

    def _item_complete(self):
        item = self._active_item
        self._active_item = None
        self._completed += 1
        if self._queue:
            self._set_status("item_complete", f"Reached and pointed at {item}")
        else:
            self._set_status(
                "complete", f"Completed all {self._total} requested item(s)"
            )

    def _fractional_target(self, maximum_parameter, fraction_parameter):
        maximum = float(self.get_parameter(maximum_parameter).value)
        fraction = float(self.get_parameter(fraction_parameter).value)
        return maximum * min(1.0, max(0.0, fraction))

    def _is_active(self):
        return bool(
            self._queue
            or self._goal_handle is not None
            or self._goal_request_pending
            or self._turn_needed
            or self._arm_goal_handle is not None
            or self._arm_request_pending
            or self._hold_timer is not None
            or self._active_item is not None
        )

    def _abort(self, detail):
        self._queue.clear()
        if self._hold_timer is not None:
            self._hold_timer.cancel()
            self.destroy_timer(self._hold_timer)
            self._hold_timer = None
        self._goal_handle = None
        self._goal_request_pending = False
        self._publish_turn_stop()
        self._turn_needed = False
        self._turn_started_ns = None
        self._last_turn_status_ns = None
        self._pointing_yaw = None
        self._arm_goal_handle = None
        self._arm_request_pending = False
        self._active_item = None
        self._stopping = False
        self._set_status("failed", detail)
        self.get_logger().error(detail)

    def _stop_callback(self, _request, response):
        was_active = self._is_active()
        self._queue.clear()
        self._stopping = True
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._publish_turn_stop()
        self._turn_needed = False
        self._turn_started_ns = None
        self._last_turn_status_ns = None
        self._pointing_yaw = None
        if self._arm_goal_handle is not None:
            self._arm_goal_handle.cancel_goal_async()
        if self._hold_timer is not None:
            self._hold_timer.cancel()
            self.destroy_timer(self._hold_timer)
            self._hold_timer = None
        self._active_item = None
        self._set_status("stopped", "Stopped by service request")
        response.success = was_active
        response.message = "Travel stopped" if was_active else "No travel was active"
        return response

    def _status_callback(self, _request, response):
        response.success = self._state in {
            "queued",
            "waiting_for_nav2",
            "waiting_for_arm",
            "waiting_for_tf",
            "navigating",
            "waiting_to_turn",
            "turning",
            "raising_arm",
            "aiming",
            "pointing",
            "retracting",
            "stowing",
            "item_complete",
        }
        response.message = f"{self._state}: {self._detail}"
        return response

    def _set_status(self, state, detail):
        self._state = state
        self._detail = detail
        self._publish_status()

    def _publish_status(self):
        message = String()
        message.data = f"{self._state}: {self._detail}"
        self._status_publisher.publish(message)
        self.get_logger().info(message.data)


def main(args=None):
    rclpy.init(args=args)
    node = KitchenItemTraveler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
