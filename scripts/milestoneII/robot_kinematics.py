#!/usr/bin/env python3

import copy
from pathlib import Path
import tempfile
import time
import warnings
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from ikpy.chain import Chain
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
import xacro

from cs685_fall26_materials.action import ExecuteCartesianPoint, ExecuteJointPose


ACTION_NAME = "/milestoneII/execute_joint_pose"
IK_ACTION_NAME = "/milestoneII/execute_cartesian_point"
TRAJECTORY_ACTION_NAME = "/stretch_controller/follow_joint_trajectory"
JOINT_STATE_TOPIC = "/stretch/joint_states"
TRAJECTORY_DURATION_SECONDS = 3.0
BASE_FRAME = "base_link"
MAP_FRAME = "map"
END_EFFECTOR_FRAME = "link_grasp_center"
BASE_TRANSLATION_JOINT = "translate_mobile_base"
LIFT_JOINT = "joint_lift"
ARM_JOINT = "wrist_extension"
WRIST_YAW_JOINT = "joint_wrist_yaw"
WRIST_PITCH_JOINT = "joint_wrist_pitch"
WRIST_ROLL_JOINT = "joint_wrist_roll"
ARM_SEGMENT_JOINTS = [f"joint_arm_l{index}" for index in range(4)]
IK_ACTIVE_JOINTS = {
    BASE_TRANSLATION_JOINT,
    LIFT_JOINT,
    *ARM_SEGMENT_JOINTS,
    WRIST_YAW_JOINT,
    WRIST_PITCH_JOINT,
    WRIST_ROLL_JOINT,
}
IK_POSITION_TOLERANCE = 0.02


class RobotKinematics(Node):
    """Receives joint-pose requests for the student kinematics pipeline."""

    def __init__(self):
        super().__init__("robot_kinematics")
        self._callback_group = ReentrantCallbackGroup()
        self._ik_urdf_path = None
        self._ik_chain = self._create_ik_chain()
        self._joint_positions = {}
        self._joint_state_subscription = self.create_subscription(
            JointState,
            JOINT_STATE_TOPIC,
            self._joint_state_callback,
            10,
            callback_group=self._callback_group,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            TRAJECTORY_ACTION_NAME,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            ExecuteJointPose,
            ACTION_NAME,
            self._execute_joint_pose,
            callback_group=self._callback_group,
        )
        self._ik_action_server = ActionServer(
            self,
            ExecuteCartesianPoint,
            IK_ACTION_NAME,
            self._execute_cartesian_point,
            callback_group=self._callback_group,
        )
        self.get_logger().info(f"Ready for joint-pose goals on {ACTION_NAME}")
        self.get_logger().info(f"Ready for Cartesian-point goals on {IK_ACTION_NAME}")

    def _joint_state_callback(self, message):
        self._joint_positions.update(zip(message.name, message.position))

    async def _execute_joint_pose(self, goal_handle):
        joint_names = list(goal_handle.request.joint_names)
        positions = list(goal_handle.request.positions)
        self.get_logger().info(
            f"Received a joint-pose goal containing {len(joint_names)} joints"
        )

        if not joint_names or len(joint_names) != len(positions):
            goal_handle.abort()
            result = ExecuteJointPose.Result()
            result.success = False
            result.message = "Joint names and positions must be nonempty and equal length"
            return result

        success, message = await self._execute_forward_kinematics(
            joint_names,
            positions,
            lambda status: self._publish_joint_feedback(goal_handle, status),
            goal_handle.request.duration_seconds,
        )

        result = ExecuteJointPose.Result()
        result.success = success
        result.message = message
        if success:
            goal_handle.succeed()
            self.get_logger().info(message)
        else:
            goal_handle.abort()
        return result

    async def _execute_cartesian_point(self, goal_handle):
        target = goal_handle.request.target
        result = ExecuteCartesianPoint.Result()

        if target.header.frame_id != MAP_FRAME:
            goal_handle.abort()
            result.success = False
            result.message = f"Target point must be expressed in {MAP_FRAME}"
            return result

        self._publish_ik_feedback(goal_handle, f"Transforming target into {BASE_FRAME}")
        try:
            map_to_base = self._tf_buffer.lookup_transform(
                BASE_FRAME,
                MAP_FRAME,
                Time(),
                timeout=Duration(seconds=1.0),
            )
            base_target = do_transform_point(target, map_to_base)
        except TransformException as error:
            goal_handle.abort()
            result.success = False
            result.message = (
                f"Cannot transform target from {MAP_FRAME} into {BASE_FRAME}: {error}"
            )
            return result

        self._publish_ik_feedback(goal_handle, "Computing inverse kinematics")
        if not await self._wait_for_joint_positions():
            goal_handle.abort()
            result.success = False
            result.message = f"Timed out waiting for IK joints on {JOINT_STATE_TOPIC}"
            return result

        try:
            joint_names, positions = self._inverse_kinematics(
                base_target.point.x,
                base_target.point.y,
                base_target.point.z,
            )
        except (RuntimeError, ValueError) as error:
            goal_handle.abort()
            result.success = False
            result.message = f"Inverse kinematics failed: {error}"
            return result

        result.joint_names = joint_names
        result.positions = positions
        self.get_logger().info(
            "IK solution: "
            + ", ".join(
                f"{name}={position:.3f}" for name, position in zip(joint_names, positions)
            )
        )

        success, message = await self._execute_forward_kinematics(
            joint_names,
            positions,
            lambda status: self._publish_ik_feedback(goal_handle, status),
            goal_handle.request.duration_seconds,
        )
        result.success = success
        result.message = message
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _inverse_kinematics(self, target_x, target_y, target_z):
        """Solve the complete Cartesian Stretch chain."""
        # TODO: Milestone II — compute joint values for the target Cartesian position
        raise RuntimeError("Inverse kinematics is not implemented")

    def _create_ik_chain(self):
        """Create the tutorial's Cartesian chain from Stretch's installed URDF."""
        description_directory = Path(
            get_package_share_directory("stretch_description")
        ) / "urdf"
        calibrated_urdf = description_directory / "stretch.urdf"
        description_file = (
            calibrated_urdf
            if calibrated_urdf.is_file()
            else description_directory
            / "stretch_description_SE3_eoa_wrist_dw3_tool_sg3.xacro"
        )
        expanded_urdf = xacro.process_file(str(description_file)).toxml()
        ik_urdf = self._make_cartesian_ik_urdf(expanded_urdf)

        temporary_urdf = tempfile.NamedTemporaryFile(
            mode="w",
            suffix=".urdf",
            prefix="stretch_cartesian_ik_",
            delete=False,
        )
        with temporary_urdf:
            temporary_urdf.write(ik_urdf)
        self._ik_urdf_path = Path(temporary_urdf.name)

        with warnings.catch_warnings():
            warnings.filterwarnings(
                "ignore",
                message="Joint .* is of type: fixed, but has an 'axis' attribute",
            )
            warnings.filterwarnings(
                "ignore",
                message="Link .* is of type 'fixed' but set as active",
            )
            chain = Chain.from_urdf_file(
                str(self._ik_urdf_path),
                base_elements=[BASE_FRAME],
                base_element_type="link",
            )
        chain.active_links_mask = np.array(
            [link.name in IK_ACTIVE_JOINTS for link in chain.links],
            dtype=bool,
        )
        missing = IK_ACTIVE_JOINTS.difference(link.name for link in chain.links)
        if missing:
            raise RuntimeError(f"IK URDF is missing joints: {sorted(missing)}")
        return chain

    @staticmethod
    def _make_cartesian_ik_urdf(expanded_urdf):
        """Extract base-to-grasp chain and insert a virtual base x joint."""
        source_root = ET.fromstring(expanded_urdf)
        links = {element.get("name"): element for element in source_root.findall("link")}
        joints_by_child = {}
        for joint in source_root.findall("joint"):
            child = joint.find("child")
            if child is not None:
                joints_by_child[child.get("link")] = joint

        chain_joints = []
        link_name = END_EFFECTOR_FRAME
        while link_name != BASE_FRAME:
            joint = joints_by_child.get(link_name)
            if joint is None:
                raise RuntimeError(
                    f"URDF has no chain from {BASE_FRAME} to {END_EFFECTOR_FRAME}"
                )
            chain_joints.append(joint)
            link_name = joint.find("parent").get("link")
        chain_joints.reverse()

        output = ET.Element("robot", {"name": "stretch_cartesian_ik"})
        output.append(copy.deepcopy(links[BASE_FRAME]))
        virtual_link_name = "link_base_translation"
        virtual_joint = ET.SubElement(
            output,
            "joint",
            {"name": BASE_TRANSLATION_JOINT, "type": "prismatic"},
        )
        ET.SubElement(virtual_joint, "parent", {"link": BASE_FRAME})
        ET.SubElement(virtual_joint, "child", {"link": virtual_link_name})
        ET.SubElement(virtual_joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(virtual_joint, "axis", {"xyz": "1 0 0"})
        ET.SubElement(
            virtual_joint,
            "limit",
            {"lower": "-1.0", "upper": "1.0", "effort": "100", "velocity": "1.0"},
        )
        ET.SubElement(output, "link", {"name": virtual_link_name})

        for index, source_joint in enumerate(chain_joints):
            joint = copy.deepcopy(source_joint)
            if index == 0:
                joint.find("parent").set("link", virtual_link_name)
            output.append(joint)
            child_name = joint.find("child").get("link")
            output.append(copy.deepcopy(links[child_name]))
        return ET.tostring(output, encoding="unicode")

    async def _wait_for_joint_positions(self, timeout_seconds=2.0):
        """Allow joint-state callbacks to run while an early IK goal waits."""
        def positions_available():
            required = {
                LIFT_JOINT,
                ARM_JOINT,
                WRIST_YAW_JOINT,
                WRIST_PITCH_JOINT,
                WRIST_ROLL_JOINT,
            }
            return required.issubset(self._joint_positions)

        if positions_available():
            return True

        ready = Future()
        deadline = time.monotonic() + timeout_seconds

        def check_readiness():
            if ready.done():
                return
            if positions_available():
                ready.set_result(True)
            elif time.monotonic() >= deadline:
                ready.set_result(False)

        timer = self.create_timer(
            0.05,
            check_readiness,
            callback_group=self._callback_group,
        )
        try:
            return await ready
        finally:
            self.destroy_timer(timer)

    @staticmethod
    def _require_within_limits(joint_name, value, limits):
        lower, upper = limits
        if not lower <= value <= upper:
            raise ValueError(
                f"{joint_name} solution {value:.3f} is outside [{lower:.3f}, {upper:.3f}]"
            )

    async def _execute_forward_kinematics(
        self,
        joint_names,
        positions,
        publish_feedback,
        duration_seconds=0.0,
    ):
        """Package and execute one joint pose for both the FK and IK actions."""
        # TODO: Milestone II — execute the requested joint pose
        return False, "Forward kinematics is not implemented"

    @staticmethod
    def _publish_joint_feedback(goal_handle, status):
        feedback = ExecuteJointPose.Feedback()
        feedback.status = status
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _publish_ik_feedback(goal_handle, status):
        feedback = ExecuteCartesianPoint.Feedback()
        feedback.status = status
        goal_handle.publish_feedback(feedback)

    def destroy_node(self):
        self._action_server.destroy()
        self._ik_action_server.destroy()
        self._trajectory_client.destroy()
        if self._ik_urdf_path is not None:
            self._ik_urdf_path.unlink(missing_ok=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotKinematics()
    executor = MultiThreadedExecutor(num_threads=2)
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
