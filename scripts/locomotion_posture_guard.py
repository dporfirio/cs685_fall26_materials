#!/usr/bin/env python3

"""Gate base velocity until Stretch's arm is in a safe travel posture."""

import math

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint


class LocomotionPostureGuard(Node):
    JOINTS = (
        "wrist_extension",
        "joint_wrist_pitch",
        "joint_wrist_roll",
        "joint_wrist_yaw",
    )

    def __init__(self):
        super().__init__("locomotion_posture_guard")
        self.declare_parameter("input_cmd_vel", "/stretch/cmd_vel")
        self.declare_parameter("fallback_input_cmd_vel", "/cmd_vel")
        self.declare_parameter("output_cmd_vel", "/stretch/cmd_vel_guarded")
        self.declare_parameter("joint_states_topic", "/stretch/joint_states")
        self.declare_parameter("scan_topic", "/scan_filtered")
        self.declare_parameter("wrist_extension", 0.0)
        self.declare_parameter("wrist_pitch", -0.4)
        self.declare_parameter("wrist_roll", 0.0)
        self.declare_parameter("wrist_yaw", 2.8)
        self.declare_parameter("extension_tolerance", 0.01)
        self.declare_parameter("angular_tolerance", 0.08)
        self.declare_parameter("command_timeout", 0.35)
        self.declare_parameter("minimum_velocity", 1e-4)
        self.declare_parameter("travel_posture_duration", 2.0)
        self.declare_parameter("trajectory_retry_period", 1.0)
        self.declare_parameter("startup_clearance", 0.6)
        self.declare_parameter("startup_escape_distance", 0.25)
        self.declare_parameter("startup_escape_linear_speed", 0.1)
        self.declare_parameter("startup_escape_angular_speed", 0.4)
        self.declare_parameter("startup_scan_angle_offset", math.pi)

        self.targets = {
            "wrist_extension": float(
                self.get_parameter("wrist_extension").value
            ),
            "joint_wrist_pitch": float(
                self.get_parameter("wrist_pitch").value
            ),
            "joint_wrist_roll": float(self.get_parameter("wrist_roll").value),
            "joint_wrist_yaw": float(self.get_parameter("wrist_yaw").value),
        }
        self.positions = {}
        self.latest_command = Twist()
        self.last_command_ns = None
        self.motion_requested = False
        self.initial_posture_established = False
        self.latest_scan = None
        self.clearance_complete = False
        self.escape_phase = "checking"
        self.escape_end_ns = None
        self.escape_turn_sign = 0.0
        self.escape_drive_sign = 1.0
        self.stow_pending = False
        self.stow_complete = False
        self.trajectory_pending = False
        self.goal_handle = None
        self.last_trajectory_request_ns = None
        self.state = "waiting_for_joint_states"

        self.velocity_publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_cmd_vel").value), 10
        )
        self.status_publisher = self.create_publisher(
            String, "/locomotion_posture_guard/status", 10
        )
        input_cmd_vel = str(self.get_parameter("input_cmd_vel").value)
        self.create_subscription(
            Twist,
            input_cmd_vel,
            self._velocity_callback,
            10,
        )
        fallback_input = str(
            self.get_parameter("fallback_input_cmd_vel").value
        )
        if fallback_input and fallback_input != input_cmd_vel:
            self.create_subscription(
                Twist,
                fallback_input,
                self._velocity_callback,
                10,
            )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._joint_callback,
            10,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/stretch_controller/follow_joint_trajectory",
        )
        self.stow_client = self.create_client(Trigger, "/stow_the_robot")
        self.create_timer(0.05, self._safety_tick)
        self._publish_status("Waiting to establish travel posture")

    def _joint_callback(self, message):
        for name, position in zip(message.name, message.position):
            if name in self.JOINTS:
                self.positions[name] = float(position)

    def _scan_callback(self, message):
        self.latest_scan = message

    def _velocity_callback(self, message):
        self.latest_command = message
        self.last_command_ns = self.get_clock().now().nanoseconds
        self.motion_requested = self._is_motion_command(message)

        if not self.clearance_complete:
            # The startup state machine exclusively owns the guarded output.
            # Publishing a stop for each incoming Nav2 command would overwrite
            # its turn/drive command and prevent the escape maneuver.
            return

        if not self.motion_requested:
            self.velocity_publisher.publish(message)
            self._publish_status("Stopped; arm posture is unrestricted")
            return

        if self._posture_is_safe():
            self.velocity_publisher.publish(message)
            self._publish_status("Locomotion enabled; protected posture confirmed")
        else:
            if (
                self.initial_posture_established
                and not self.stow_pending
                and not self.trajectory_pending
            ):
                # The arm may be freely repositioned while stopped. A later
                # request to move must begin with a fresh native stow.
                self.stow_complete = False
            self._publish_stop()
            self._request_safe_posture()
            self._publish_status(self._blocked_posture_detail())

    def _safety_tick(self):
        if not self.clearance_complete:
            self._run_startup_clearance()
            return

        if (
            self._posture_is_safe()
            and self.stow_complete
            and not self.stow_pending
            and not self.trajectory_pending
        ):
            self.initial_posture_established = True
        elif self.motion_requested or not self.initial_posture_established:
            if self.motion_requested:
                self._publish_stop()
            self._request_safe_posture()

        if not self.motion_requested:
            return
        now_ns = self.get_clock().now().nanoseconds
        timeout = float(self.get_parameter("command_timeout").value)
        if (
            self.last_command_ns is None
            or (now_ns - self.last_command_ns) / 1e9 > timeout
        ):
            self.motion_requested = False
            self._publish_stop()
            self._publish_status("Stopped after velocity-command timeout")
            return

        if not self._posture_is_safe():
            self._publish_stop()
            self._publish_status(self._blocked_posture_detail())

    def _run_startup_clearance(self):
        """Back away from nearby geometry before moving the arm to stow."""
        now_ns = self.get_clock().now().nanoseconds

        if self.escape_phase == "checking":
            if self.latest_scan is None:
                self._publish_stop()
                self._publish_status("Waiting for lidar before native stow")
                return

            scan = self.latest_scan
            valid = []
            # stretch_mujoco's lidar body is rotated 180 degrees, while its
            # LaserScan publisher labels index zero as +X. Compensate for that
            # simulator frame mismatch before deriving an escape direction.
            angle = scan.angle_min + float(
                self.get_parameter("startup_scan_angle_offset").value
            )
            for distance in scan.ranges:
                if (
                    math.isfinite(distance)
                    and scan.range_min <= distance <= scan.range_max
                ):
                    valid.append((float(distance), angle))
                angle += scan.angle_increment

            threshold = float(self.get_parameter("startup_clearance").value)
            if not valid or min(distance for distance, _ in valid) >= threshold:
                self.clearance_complete = True
                self._publish_stop()
                self._publish_status("Startup clearance is safe; beginning native stow")
                return

            # Sum repulsive vectors from all nearby returns rather than reacting
            # to one noisy ray. The result points away from surrounding geometry.
            repel_x = 0.0
            repel_y = 0.0
            for distance, ray_angle in valid:
                if distance >= threshold:
                    continue
                weight = (threshold - distance) / threshold
                repel_x -= weight * math.cos(ray_angle)
                repel_y -= weight * math.sin(ray_angle)
            away_angle = math.atan2(repel_y, repel_x)

            # Driving backward can reach the same escape vector with less turn.
            self.escape_drive_sign = 1.0
            turn_angle = self._normalize_angle(away_angle)
            if abs(turn_angle) > math.pi / 2.0:
                self.escape_drive_sign = -1.0
                turn_angle = self._normalize_angle(away_angle - math.pi)

            angular_speed = float(
                self.get_parameter("startup_escape_angular_speed").value
            )
            if abs(turn_angle) > 0.05 and angular_speed > 0.0:
                self.escape_turn_sign = math.copysign(1.0, turn_angle)
                self.escape_end_ns = now_ns + int(
                    abs(turn_angle) / angular_speed * 1e9
                )
                self.escape_phase = "turning"
            else:
                self._begin_escape_drive(now_ns)

        if self.escape_phase == "turning":
            if now_ns < self.escape_end_ns:
                command = Twist()
                command.angular.z = self.escape_turn_sign * float(
                    self.get_parameter("startup_escape_angular_speed").value
                )
                self.velocity_publisher.publish(command)
                self._publish_status("Creating clearance before native stow: turning")
                return
            self._publish_stop()
            self._begin_escape_drive(now_ns)

        if self.escape_phase == "driving":
            if now_ns < self.escape_end_ns:
                command = Twist()
                command.linear.x = self.escape_drive_sign * float(
                    self.get_parameter("startup_escape_linear_speed").value
                )
                self.velocity_publisher.publish(command)
                self._publish_status("Creating clearance before native stow: moving away")
                return
            self._publish_stop()
            self.escape_phase = "done"
            self.clearance_complete = True
            self._publish_status("Startup escape complete; beginning native stow")

    def _begin_escape_drive(self, now_ns):
        speed = float(self.get_parameter("startup_escape_linear_speed").value)
        distance = float(self.get_parameter("startup_escape_distance").value)
        self.escape_end_ns = now_ns + int(distance / max(speed, 1e-3) * 1e9)
        self.escape_phase = "driving"

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _posture_is_safe(self):
        if any(joint not in self.positions for joint in self.JOINTS):
            return False
        extension_tolerance = float(
            self.get_parameter("extension_tolerance").value
        )
        angular_tolerance = float(self.get_parameter("angular_tolerance").value)
        for joint in self.JOINTS:
            tolerance = (
                extension_tolerance
                if joint == "wrist_extension"
                else angular_tolerance
            )
            if abs(self.positions[joint] - self.targets[joint]) > tolerance:
                return False
        return True

    def _request_safe_posture(self):
        if self.stow_pending or self.trajectory_pending or self._posture_is_safe():
            return
        missing = [joint for joint in self.JOINTS if joint not in self.positions]
        if missing:
            # Fail closed until interpolation has a measured start position for
            # every protected joint. The next joint-state or velocity callback
            # will allow another attempt once feedback is complete.
            return
        now_ns = self.get_clock().now().nanoseconds
        retry_period = float(
            self.get_parameter("trajectory_retry_period").value
        )
        if (
            self.last_trajectory_request_ns is not None
            and (now_ns - self.last_trajectory_request_ns) / 1e9 < retry_period
        ):
            return
        if not self.stow_complete:
            if not self.stow_client.service_is_ready():
                self._publish_status("Waiting for native stow service")
                return
            self.stow_pending = True
            self.last_trajectory_request_ns = now_ns
            future = self.stow_client.call_async(Trigger.Request())
            future.add_done_callback(self._stow_result)
            return

        # Native stow establishes extension=0, pitch=-0.4, roll=0 and
        # yaw=3.14 using MuJoCo's supported keyframe. Only correct yaw to the
        # desired not-fully-stowed 2.8-rad travel angle afterward.
        if not self.trajectory_client.server_is_ready():
            self._publish_status("Waiting for wrist-yaw trajectory server")
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["joint_wrist_yaw"]
        duration = max(
            0.1, float(self.get_parameter("travel_posture_duration").value)
        )
        point = JointTrajectoryPoint()
        point.positions = [self.targets["joint_wrist_yaw"]]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int(
            (duration - int(duration)) * 1e9
        )
        goal.trajectory.points.append(point)
        self.trajectory_pending = True
        self.last_trajectory_request_ns = now_ns
        future = self.trajectory_client.send_goal_async(goal)
        future.add_done_callback(self._trajectory_response)

    def _stow_result(self, future):
        self.stow_pending = False
        try:
            response = future.result()
        except Exception as error:
            self._publish_status(f"Native stow request failed: {error}")
            return
        if not response.success:
            self._publish_status(f"Native stow failed: {response.message}")
            return
        self.stow_complete = True
        # The retry timestamp belongs to the completed stow request. Clear it
        # so the required yaw correction is not delayed or mistaken for a
        # repeated stow attempt.
        self.last_trajectory_request_ns = None
        self._publish_status("Native stow complete; applying travel yaw")
        self._request_safe_posture()

    def _trajectory_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.trajectory_pending = False
            self._publish_status(f"Travel-posture request failed: {error}")
            return
        if not goal_handle.accepted:
            self.trajectory_pending = False
            self._publish_status("Travel-posture trajectory was rejected")
            return
        self.goal_handle = goal_handle
        result = goal_handle.get_result_async()
        result.add_done_callback(self._trajectory_result)

    def _trajectory_result(self, future):
        self.trajectory_pending = False
        self.goal_handle = None
        try:
            future.result()
        except Exception as error:
            self.stow_complete = False
            self._publish_status(f"Travel-posture trajectory failed: {error}")
            return
        if self.motion_requested and self._posture_is_safe():
            self._publish_status("Travel posture reached; awaiting current velocity command")
        elif not self._posture_is_safe():
            # Re-run the reliable native stow sequence rather than repeatedly
            # issuing corrections from an unknown mechanical configuration.
            self.stow_complete = False

    def _blocked_posture_detail(self):
        if self.stow_pending:
            return "Native stow in progress"
        if self.trajectory_pending:
            return "Travel-yaw correction in progress"
        missing = [joint for joint in self.JOINTS if joint not in self.positions]
        if missing:
            return "Locomotion blocked; missing joint feedback: " + ", ".join(missing)
        errors = ", ".join(
            f"{joint}={self.positions[joint] - self.targets[joint]:+.3f}"
            for joint in self.JOINTS
        )
        return f"Locomotion blocked while establishing travel posture; errors: {errors}"

    def _is_motion_command(self, message):
        threshold = float(self.get_parameter("minimum_velocity").value)
        values = (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
        return any(math.fabs(value) > threshold for value in values)

    def _publish_stop(self):
        self.velocity_publisher.publish(Twist())

    def _publish_status(self, detail):
        state = "moving" if self.motion_requested and self._posture_is_safe() else "blocked"
        if not self.motion_requested:
            state = "stopped"
        message = String()
        message.data = f"state={state}; {detail}"
        if message.data == self.state:
            return
        self.state = message.data
        self.status_publisher.publish(message)
        self.get_logger().info(message.data)


def main():
    rclpy.init()
    node = LocomotionPostureGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.velocity_publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
