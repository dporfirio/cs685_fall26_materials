#!/usr/bin/env python3

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from cs685_fall26_materials.action import ExecuteCartesianPoint, ExecuteJointPose


ACTION_NAME = "/milestoneII/execute_joint_pose"
IK_ACTION_NAME = "/milestoneII/execute_cartesian_point"
IK_TARGET_TOPIC = "/milestoneII/ik_target"
FORWARD_KINEMATICS_DURATION_SECONDS = 3.0
INVERSE_KINEMATICS_DURATION_SECONDS = 6.0


class MilestoneIIController(Node):
    """Sequences joint targets through the robot kinematics node."""

    def __init__(self):
        super().__init__("milestoneII_controller")
        self._joint_pose_client = ActionClient(
            self, ExecuteJointPose, ACTION_NAME
        )
        self._cartesian_point_client = ActionClient(
            self, ExecuteCartesianPoint, IK_ACTION_NAME
        )
        self._ik_target_publisher = self.create_publisher(
            PointStamped,
            IK_TARGET_TOPIC,
            1,
        )
        self._active_goal = None
        self._connection_announced = False
        self._connection_timer = self.create_timer(0.5, self._check_connection)

        self._sequence_started = False
        self._start_service = self.create_service(
            Trigger,
            "/milestoneII/initiate_sequence",
            self._initiate_sequence_callback,
        )

        # hard-coded joint/end-effector poses
        self.joints = ["joint_lift", "wrist_extension", "joint_wrist_yaw",
                       "joint_wrist_pitch", "joint_wrist_roll", "joint_gripper_finger_left"]
        self.fw_kinematics_sequence = [
            [0.2, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.5, 0.5, 3.5, -0.3, 0.0, 0.1],
            [0.8, 0.0, -0.5, 0.0, 0.0, 0.0],
        ]
        self.inv_kinematics_sequence = [
            [3.097, -0.950, 0.600],
            [2.597, -0.850, 0.750],
        ]

    def _initiate_sequence_callback(self, request, response):
        del request

        if self._sequence_started:
            response.success = False
            response.message = "The sequence has already been started"
            return response

        if not self._joint_pose_client.server_is_ready():
            response.success = False
            response.message = "The robot kinematics action server is not ready"
            return response
        if (
            self.inv_kinematics_sequence
            and not self._cartesian_point_client.server_is_ready()
        ):
            response.success = False
            response.message = "The inverse-kinematics action server is not ready"
            return response

        self._sequence_started = True
        self.initiate_sequence()

        response.success = True
        response.message = "Joint-pose sequence initiated"
        return response
    
    def initiate_sequence(self):
        if self.fw_kinematics_sequence:
            self.send_joint_pose(
                self.joints,
                self.fw_kinematics_sequence[0],
            )
            return
        if self.inv_kinematics_sequence:
            self.send_cartesian_point(self.inv_kinematics_sequence[0])
            return
        self.get_logger().info("There are no kinematics goals to execute")

    def _check_connection(self):
        if not self._joint_pose_client.server_is_ready():
            return
        if not self._connection_announced:
            self.get_logger().info(
                f"Connected to the robot kinematics action server at {ACTION_NAME}"
            )
            self._connection_announced = True
        self._connection_timer.cancel()

    def send_joint_pose(
        self,
        joint_names,
        positions,
        duration_seconds=FORWARD_KINEMATICS_DURATION_SECONDS,
    ):
        """Send one joint batch; the next batch belongs in the result callback."""
        if self._active_goal is not None:
            raise RuntimeError("A joint-pose goal is already active")
        if len(joint_names) != len(positions):
            raise ValueError("joint_names and positions must have the same length")
        if not self._joint_pose_client.server_is_ready():
            self.get_logger().warning("Robot kinematics action server is not ready")
            return False

        goal = ExecuteJointPose.Goal()
        goal.joint_names = list(joint_names)
        goal.positions = [float(position) for position in positions]
        goal.duration_seconds = float(duration_seconds)

        self._active_goal = "pending"
        future = self._joint_pose_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        future.add_done_callback(self._goal_response_callback)
        return True

    def _feedback_callback(self, feedback_message):
        self.get_logger().info(feedback_message.feedback.status)

    def _goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self._active_goal = None
            self.get_logger().error(f"Joint-pose request failed: {error}")
            return
        if not goal_handle.accepted:
            self._active_goal = None
            self.get_logger().error("Robot kinematics rejected the joint-pose goal")
            return

        self._active_goal = goal_handle
        goal_handle.get_result_async().add_done_callback(self._result_callback)

    def _result_callback(self, future):
        self._active_goal = None
        try:
            wrapped_result = future.result()
        except Exception as error:
            self.get_logger().error(f"Could not receive joint-pose result: {error}")
            return

        result = wrapped_result.result
        if (
            wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            and result.success
        ):
            self.get_logger().info(f"Joint-pose goal complete: {result.message}")
            # next pose
            self.fw_kinematics_sequence.pop(0)
            if len(self.fw_kinematics_sequence) > 0:
                self.send_joint_pose(self.joints, self.fw_kinematics_sequence[0])
            elif self.inv_kinematics_sequence:
                self.send_cartesian_point(self.inv_kinematics_sequence[0])
            return
        self.get_logger().error(
            f"Joint-pose goal failed (status={wrapped_result.status}): "
            f"{result.message}"
        )

    def send_cartesian_point(
        self,
        xyz,
        duration_seconds=INVERSE_KINEMATICS_DURATION_SECONDS,
    ):
        """Send one point, expressed in map, to the IK action server."""
        if self._active_goal is not None:
            raise RuntimeError("A kinematics goal is already active")
        if len(xyz) != 3:
            raise ValueError("A Cartesian point must contain x, y, and z")
        if not self._cartesian_point_client.server_is_ready():
            self.get_logger().warning("Inverse-kinematics action server is not ready")
            return False

        goal = ExecuteCartesianPoint.Goal()
        goal.target = PointStamped()
        goal.target.header.frame_id = "map"
        goal.target.header.stamp = self.get_clock().now().to_msg()
        goal.target.point.x, goal.target.point.y, goal.target.point.z = map(float, xyz)
        goal.duration_seconds = float(duration_seconds)

        self._ik_target_publisher.publish(goal.target)
        self._active_goal = "pending"
        future = self._cartesian_point_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        future.add_done_callback(self._ik_goal_response_callback)
        return True

    def _ik_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self._active_goal = None
            self._clear_ik_target()
            self.get_logger().error(f"Cartesian-point request failed: {error}")
            return
        if not goal_handle.accepted:
            self._active_goal = None
            self._clear_ik_target()
            self.get_logger().error("Robot kinematics rejected the Cartesian-point goal")
            return

        self._active_goal = goal_handle
        goal_handle.get_result_async().add_done_callback(self._ik_result_callback)

    def _ik_result_callback(self, future):
        self._active_goal = None
        self._clear_ik_target()
        try:
            wrapped_result = future.result()
        except Exception as error:
            self.get_logger().error(f"Could not receive Cartesian-point result: {error}")
            return

        result = wrapped_result.result
        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED and result.success:
            self.get_logger().info(
                f"Cartesian-point goal complete: {result.message}; solution="
                + ", ".join(
                    f"{name}={position:.3f}"
                    for name, position in zip(result.joint_names, result.positions)
                )
            )
            self.inv_kinematics_sequence.pop(0)
            if self.inv_kinematics_sequence:
                self.send_cartesian_point(self.inv_kinematics_sequence[0])
            return
        self.get_logger().error(
            f"Cartesian-point goal failed (status={wrapped_result.status}): "
            f"{result.message}"
        )

    def _clear_ik_target(self):
        reset = PointStamped()
        reset.header.stamp = self.get_clock().now().to_msg()
        self._ik_target_publisher.publish(reset)

    def destroy_node(self):
        self._clear_ik_target()
        self._joint_pose_client.destroy()
        self._cartesian_point_client.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MilestoneIIController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
