"""Stretch MuJoCo driver with course-owned initial poses."""

import math
import threading
import time

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped
import mujoco.viewer
import mujoco._functions
import numpy as np
import stretch_mujoco_driver.stretch_mujoco_driver as driver
import stretch_mujoco_driver.joint_trajectory_server as trajectory_server
import stretch_mujoco.stretch_mujoco_simulator as simulator
from stretch_mujoco.datamodels.status_command import CommandCoordinateFrameArrowsViz
from stretch_mujoco.enums.actuators import Actuators
from stretch_mujoco.mujoco_server_passive import MujocoServerPassive
from stretch_mujoco.stretch_mujoco_simulator import StretchMujocoSimulator


ROBOT_POSITION = [3.0, -1.4, 0.0]
ROBOT_QUATERNION = [0.0, 0.0, 0.0, -1.0]
CAMERA_LOOKAT = [3.0, -1.4, 0.7]
CAMERA_DISTANCE = 2.0
CAMERA_AZIMUTH = 0.0
CAMERA_ELEVATION = -20.0
TRAJECTORY_RATE_HZ = 30.0
BASE_POSITION_TOLERANCE = 0.01
BASE_COMPLETION_TIMEOUT_SECONDS = 1.0
BASE_MINIMUM_SPEED = 0.05
BASE_MAXIMUM_SPEED = 0.30
IK_TARGET_TOPIC = "/milestoneII/ik_target"
IK_TARGET_RADIUS = 0.04
IK_TARGET_SENTINEL = (float("nan"), 0.0, 0.0)


class InitialPoseSimulator(StretchMujocoSimulator):
    def __init__(self, *args, **kwargs):
        kwargs["start_translation"] = ROBOT_POSITION
        kwargs["start_rotation_quat"] = ROBOT_QUATERNION
        super().__init__(*args, **kwargs)

    def wait_while_is_moving(self, actuator, *args, **kwargs):
        if actuator in (Actuators.left_wheel_vel, Actuators.right_wheel_vel):
            return None
        return super().wait_while_is_moving(actuator, *args, **kwargs)

    def set_ik_target(self, position):
        """Send an IK-target visualization command to the viewer process."""
        with self._command_lock:
            command = self.data_proxies.get_command()
            command.coordinate_frame_arrows_viz.append(
                CommandCoordinateFrameArrowsViz(
                    position=tuple(position),
                    rotation=IK_TARGET_SENTINEL,
                    trigger=True,
                )
            )
            self.data_proxies.set_command(command)

    def clear_ik_target(self):
        self.set_ik_target((float("nan"), float("nan"), float("nan")))


_launch_passive = mujoco.viewer.launch_passive


def _launch_with_initial_camera(*args, **kwargs):
    viewer = _launch_passive(*args, **kwargs)

    def apply_camera():
        # The native viewer initializes its free camera asynchronously. Apply
        # our pose briefly under its lock so that initialization cannot replace it.
        for _ in range(20):
            if not viewer.is_running():
                return
            with viewer.lock():
                viewer.cam.lookat[:] = CAMERA_LOOKAT
                viewer.cam.distance = CAMERA_DISTANCE
                viewer.cam.azimuth = CAMERA_AZIMUTH
                viewer.cam.elevation = CAMERA_ELEVATION
            time.sleep(0.05)
        print(
            "Applied initial MuJoCo camera: "
            f"lookat={CAMERA_LOOKAT}, distance={CAMERA_DISTANCE}, "
            f"azimuth={CAMERA_AZIMUTH}, elevation={CAMERA_ELEVATION}",
            flush=True,
        )

    threading.Thread(target=apply_camera, daemon=True).start()
    return viewer


class InitialCameraServer(MujocoServerPassive):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._ik_target_geom_index = None

    def _run_ui_simulation(self, show_viewer_ui):
        original = mujoco.viewer.launch_passive
        mujoco.viewer.launch_passive = _launch_with_initial_camera
        try:
            return super()._run_ui_simulation(show_viewer_ui)
        finally:
            mujoco.viewer.launch_passive = original

    def push_command(self, command_status):
        for visualization in command_status.coordinate_frame_arrows_viz.copy():
            if visualization.trigger and math.isnan(visualization.rotation[0]):
                self._update_ik_target_geometry(visualization.position)
                command_status.coordinate_frame_arrows_viz.remove(visualization)
        super().push_command(command_status)

    def _update_ik_target_geometry(self, position):
        scene = self.viewer.user_scn
        if self._ik_target_geom_index is None:
            if scene.ngeom >= scene.maxgeom:
                raise RuntimeError("MuJoCo user scene has no room for an IK target")
            self._ik_target_geom_index = scene.ngeom
            scene.ngeom += 1

        geometry = scene.geoms[self._ik_target_geom_index]
        if not np.all(np.isfinite(position)):
            geometry.rgba[:] = (0.0, 0.0, 0.0, 0.0)
            return

        mujoco._functions.mjv_initGeom(
            geometry,
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=np.array([IK_TARGET_RADIUS, 0.0, 0.0]),
            pos=np.asarray(position, dtype=float),
            mat=np.eye(3).flatten(),
            rgba=np.array([1.0, 0.0, 0.0, 1.0]),
        )


class MilestoneIIMujocoDriver(driver.StretchMujocoDriver):
    def __init__(self):
        super().__init__()
        self._ik_target_subscription = self.create_subscription(
            PointStamped,
            IK_TARGET_TOPIC,
            self._ik_target_callback,
            1,
        )

    def _ik_target_callback(self, message):
        if not message.header.frame_id:
            self.sim.clear_ik_target()
            return
        if message.header.frame_id != "map":
            self.get_logger().warning(
                f"Ignoring IK target in unsupported frame {message.header.frame_id}"
            )
            return
        self.sim.set_ik_target(
            (message.point.x, message.point.y, message.point.z)
        )


class TimedJointTrajectoryAction(trajectory_server.JointTrajectoryAction):
    """Execute MuJoCo joint waypoints according to their requested timing."""

    @staticmethod
    def _base_progress(start_status, current_status):
        """Return signed travel along the base's heading at segment start."""
        delta_x = current_status.base.x - start_status.base.x
        delta_y = current_status.base.y - start_status.base.y
        return (
            delta_x * math.cos(start_status.base.theta)
            + delta_y * math.sin(start_status.base.theta)
        )

    def _stop_base(self):
        self.node.sim.set_base_velocity(0.0, 0.0)

    @staticmethod
    def _base_velocity(remaining_distance, remaining_time):
        """Choose a usable signed speed while respecting the segment timing."""
        requested_speed = abs(remaining_distance) / max(
            remaining_time,
            1.0 / TRAJECTORY_RATE_HZ,
        )
        speed = min(
            BASE_MAXIMUM_SPEED,
            max(BASE_MINIMUM_SPEED, requested_speed),
        )
        return math.copysign(speed, remaining_distance)

    def execute_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        result = FollowJointTrajectory.Result()
        try:
            if not trajectory.points:
                raise ValueError("The trajectory contains no points")
            actuators = [
                trajectory_server.get_actuator_by_joint_names_in_command_groups(name)
                for name in trajectory.joint_names
            ]
            if any(
                actuator
                in (
                    Actuators.left_wheel_vel,
                    Actuators.right_wheel_vel,
                    Actuators.base_rotate,
                )
                for actuator in actuators
            ):
                raise ValueError(
                    "Timed interpolation does not support wheel velocity or base rotation"
                )

            status = self.node.sim.pull_status()
            start_positions = [
                0.0
                if actuator == Actuators.base_translate
                else actuator.get_position(status)
                for actuator in actuators
            ]
            previous_time = 0.0

            for point in trajectory.points:
                if len(point.positions) != len(actuators):
                    raise ValueError("Each point needs one position per joint")
                point_time = (
                    point.time_from_start.sec
                    + point.time_from_start.nanosec * 1e-9
                )
                segment_duration = point_time - previous_time
                if segment_duration < 0.0:
                    raise ValueError("Trajectory point times must be increasing")

                target_positions = list(point.positions)
                base_indices = [
                    index
                    for index, actuator in enumerate(actuators)
                    if actuator == Actuators.base_translate
                ]
                if len(base_indices) > 1:
                    raise ValueError("A trajectory may contain base translation only once")

                base_index = base_indices[0] if base_indices else None
                base_distance = 0.0
                base_start_status = None
                base_stopped = True
                base_tolerance = BASE_POSITION_TOLERANCE
                if base_index is not None:
                    base_distance = (
                        target_positions[base_index] - start_positions[base_index]
                    )
                    if abs(base_distance) > BASE_POSITION_TOLERANCE:
                        if segment_duration <= 0.0:
                            raise ValueError(
                                "Base translation requires a positive segment duration"
                            )
                        base_start_status = self.node.sim.pull_status()
                        base_velocity = self._base_velocity(
                            base_distance,
                            segment_duration,
                        )
                        base_tolerance = max(
                            BASE_POSITION_TOLERANCE,
                            abs(base_velocity) / TRAJECTORY_RATE_HZ * 1.5,
                        )
                        self.node.sim.set_base_velocity(
                            base_velocity,
                            0.0,
                        )
                        base_stopped = False

                steps = max(1, math.ceil(segment_duration * TRAJECTORY_RATE_HZ))
                segment_start = time.monotonic()
                for step in range(1, steps + 1):
                    if goal_handle.is_cancel_requested:
                        if not base_stopped:
                            self._stop_base()
                        goal_handle.canceled()
                        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                        result.error_string = "Trajectory canceled"
                        return result

                    alpha = step / steps
                    for actuator, start, target in zip(
                        actuators,
                        start_positions,
                        target_positions,
                    ):
                        desired = start + alpha * (target - start)
                        if actuator != Actuators.base_translate:
                            self.node.sim.move_to(actuator, desired)

                    if not base_stopped:
                        progress = self._base_progress(
                            base_start_status,
                            self.node.sim.pull_status(),
                        )
                        remaining_distance = base_distance - progress
                        if abs(remaining_distance) <= base_tolerance:
                            self._stop_base()
                            base_stopped = True
                        else:
                            remaining_time = max(
                                0.0,
                                segment_duration - (time.monotonic() - segment_start),
                            )
                            self.node.sim.set_base_velocity(
                                self._base_velocity(
                                    remaining_distance,
                                    remaining_time,
                                ),
                                0.0,
                            )

                    deadline = segment_start + alpha * segment_duration
                    time.sleep(max(0.0, deadline - time.monotonic()))

                if not base_stopped:
                    completion_deadline = (
                        time.monotonic() + BASE_COMPLETION_TIMEOUT_SECONDS
                    )
                    while time.monotonic() < completion_deadline:
                        progress = self._base_progress(
                            base_start_status,
                            self.node.sim.pull_status(),
                        )
                        remaining_distance = base_distance - progress
                        if abs(remaining_distance) <= base_tolerance:
                            break
                        self.node.sim.set_base_velocity(
                            self._base_velocity(
                                remaining_distance,
                                completion_deadline - time.monotonic(),
                            ),
                            0.0,
                        )
                        time.sleep(1.0 / TRAJECTORY_RATE_HZ)
                    self._stop_base()
                    base_stopped = True
                    time.sleep(1.0 / TRAJECTORY_RATE_HZ)

                    progress = self._base_progress(
                        base_start_status,
                        self.node.sim.pull_status(),
                    )
                    remaining_distance = base_distance - progress
                    if abs(remaining_distance) > base_tolerance:
                        self.node.get_logger().info(
                            "Completing the final "
                            f"{remaining_distance:.3f} m of base translation"
                        )
                        correction_start_status = self.node.sim.pull_status()
                        self.node.sim.move_by(
                            Actuators.base_translate,
                            remaining_distance,
                        )
                        self.node.sim.wait_while_is_moving(
                            Actuators.base_translate,
                            timeout=5.0,
                        )
                        correction_progress = self._base_progress(
                            correction_start_status,
                            self.node.sim.pull_status(),
                        )
                        progress += correction_progress

                    if abs(progress - base_distance) > base_tolerance:
                        raise RuntimeError(
                            "Base translation missed its target: "
                            f"requested {base_distance:.3f} m, moved {progress:.3f} m"
                        )

                start_positions = target_positions
                previous_time = point_time

            for actuator in dict.fromkeys(actuators):
                if actuator == Actuators.base_translate:
                    continue
                reached_target = self.node.sim.wait_until_at_setpoint(actuator)
                if reached_target is False:
                    result.error_code = (
                        FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                    )
                    result.error_string = f"{actuator.name} missed its target"
                    goal_handle.abort()
                    return result
        except Exception as error:
            if "base_stopped" in locals() and not base_stopped:
                self._stop_base()
            self.node.get_logger().error(f"Timed trajectory failed: {error}")
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(error)
            goal_handle.abort()
            return result

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.node.get_logger().info("Timed trajectory execution complete")
        return result


driver.StretchMujocoSimulator = InitialPoseSimulator
driver.JointTrajectoryAction = TimedJointTrajectoryAction
driver.StretchMujocoDriver = MilestoneIIMujocoDriver
simulator.MujocoServerPassive = InitialCameraServer


def main():
    driver.main()
