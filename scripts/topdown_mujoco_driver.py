#!/usr/bin/env python3

"""Stretch MuJoCo driver with its navigation camera fixed above the world."""

import math
import os

import mujoco
import numpy as np

import stretch_mujoco_driver.stretch_mujoco_driver as driver
from stretch_mujoco.enums.stretch_cameras import StretchCameras
from stretch_mujoco.enums.actuators import Actuators
from stretch_mujoco.mujoco_server_camera_manager import (
    MujocoServerCameraManagerSync,
)
from stretch_mujoco.stretch_mujoco_simulator import StretchMujocoSimulator


_original_model_generation_wizard = driver.model_generation_wizard
_original_set_camera_params = MujocoServerCameraManagerSync.set_camera_params
_original_wait_while_is_moving = StretchMujocoSimulator.wait_while_is_moving
_original_wait_until_at_setpoint = StretchMujocoSimulator.wait_until_at_setpoint


def _wait_while_is_moving_without_velocity_actuators(self, actuator, *args, **kwargs):
    """Avoid the trajectory server's invalid position wait on wheel velocity actuators."""
    if actuator in (Actuators.left_wheel_vel, Actuators.right_wheel_vel):
        return None
    return _original_wait_while_is_moving(self, actuator, *args, **kwargs)


StretchMujocoSimulator.wait_while_is_moving = (
    _wait_while_is_moving_without_velocity_actuators
)


def _wait_until_at_guard_tolerance(self, actuator, timeout=5.0, **kwargs):
    """Use tolerances consistent with the locomotion guard.

    MuJoCo's wrist yaw normally settles roughly 0.06 rad from its requested
    position.  Waiting for the driver's 0.05 default produces a false timeout;
    the guard itself accepts and continuously verifies an 0.08-rad error.
    """
    tolerance = 0.01 if actuator == Actuators.arm else 0.08
    return _original_wait_until_at_setpoint(
        self,
        actuator,
        timeout=timeout,
        position_tolerance=tolerance,
    )


StretchMujocoSimulator.wait_until_at_setpoint = _wait_until_at_guard_tolerance


def _set_camera_params_preserving_orthographic(self, camera):
    """Keep native orthographic span when Stretch initializes its renderers."""
    camera_id = self.mujoco_server.mjmodel.camera(
        camera.camera_name_in_mjcf
    ).id
    is_orthographic = bool(
        self.mujoco_server.mjmodel.cam_orthographic[camera_id]
    )
    orthographic_span = float(self.mujoco_server.mjmodel.cam_fovy[camera_id])
    _original_set_camera_params(self, camera)
    if is_orthographic:
        self.mujoco_server.mjmodel.cam_orthographic[camera_id] = 1
        self.mujoco_server.mjmodel.cam_fovy[camera_id] = orthographic_span


# The simulator uses multiprocessing "spawn" on macOS. Keeping this patch at
# module scope ensures it is also applied in the rendering child process.
MujocoServerCameraManagerSync.set_camera_params = (
    _set_camera_params_preserving_orthographic
)


def _environment_bounds(model):
    """Return tight XYZ scene bounds, ignoring MuJoCo's infinite plane."""
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    minimum = np.full(3, math.inf, dtype=float)
    maximum = np.full(3, -math.inf, dtype=float)

    for geom_id in range(model.ngeom):
        geom_type = model.geom_type[geom_id]
        if geom_type == mujoco.mjtGeom.mjGEOM_PLANE:
            continue

        center = np.asarray(data.geom_xpos[geom_id], dtype=float)
        rotation = np.asarray(data.geom_xmat[geom_id], dtype=float).reshape(3, 3)
        size = np.asarray(model.geom_size[geom_id], dtype=float)

        if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
            half_extent = np.abs(rotation) @ size
            geom_minimum = center - half_extent
            geom_maximum = center + half_extent
        elif geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
            geom_minimum = center - size[0]
            geom_maximum = center + size[0]
        elif geom_type in (
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            mujoco.mjtGeom.mjGEOM_CYLINDER,
        ):
            axial = size[1] + (size[0] if geom_type == mujoco.mjtGeom.mjGEOM_CAPSULE else 0.0)
            half_extent = np.abs(rotation) @ np.array([size[0], size[0], axial])
            geom_minimum = center - half_extent
            geom_maximum = center + half_extent
        elif geom_type == mujoco.mjtGeom.mjGEOM_ELLIPSOID:
            half_extent = np.abs(rotation) @ size
            geom_minimum = center - half_extent
            geom_maximum = center + half_extent
        elif geom_type == mujoco.mjtGeom.mjGEOM_MESH:
            mesh_id = int(model.geom_dataid[geom_id])
            vertex_start = int(model.mesh_vertadr[mesh_id])
            vertex_count = int(model.mesh_vertnum[mesh_id])
            vertices = np.asarray(
                model.mesh_vert[vertex_start : vertex_start + vertex_count],
                dtype=float,
            )
            world_vertices = vertices @ rotation.T + center
            geom_minimum = world_vertices.min(axis=0)
            geom_maximum = world_vertices.max(axis=0)
        else:
            radius = float(model.geom_rbound[geom_id])
            if not math.isfinite(radius) or radius <= 0.0:
                continue
            geom_minimum = center - radius
            geom_maximum = center + radius

        minimum = np.minimum(minimum, geom_minimum)
        maximum = np.maximum(maximum, geom_maximum)

    if not np.all(np.isfinite(minimum)) or not np.all(np.isfinite(maximum)):
        return np.array([-5.0, -5.0, 0.0]), np.array([5.0, 5.0, 3.0])
    return minimum, maximum


def _configure_overhead_camera(model):
    camera_name = StretchCameras.cam_nav_rgb.camera_name_in_mjcf
    camera_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name
    )
    if camera_id < 0:
        raise RuntimeError(f"MuJoCo camera {camera_name!r} was not found")

    minimum, maximum = _environment_bounds(model)
    center = (minimum[:2] + maximum[:2]) / 2.0
    extent = np.maximum(maximum[:2] - minimum[:2], 1.0)
    margin = float(os.environ.get("TOPDOWN_CAMERA_MARGIN", "1.0"))
    width = 800.0
    height = 600.0
    orthographic_height = max(
        margin * extent[1],
        margin * extent[0] * height / width,
    )
    # Detach the camera from the robot. MuJoCo cameras look along local -Z, so
    # the identity quaternion gives a true vertical, fixed, top-down view.
    model.cam_bodyid[camera_id] = 0
    model.cam_mode[camera_id] = mujoco.mjtCamLight.mjCAMLIGHT_FIXED
    # Frame from the XY footprint rather than the maximum initial Z. The
    # generated Stretch model contains a stray high geometry before homing;
    # using it would push the camera far away and make the kitchen tiny.
    camera_z = 4.0
    model.cam_pos[camera_id] = [center[0], center[1], camera_z]
    model.cam_quat[camera_id] = [1.0, 0.0, 0.0, 0.0]
    model.cam_orthographic[camera_id] = 1
    model.cam_fovy[camera_id] = orthographic_height

    print(
        "Configured overhead camera: "
        f"center=({center[0]:.2f}, {center[1]:.2f}), "
        f"scene={extent[0]:.2f}x{extent[1]:.2f} m, "
        f"height={camera_z:.2f} m, "
        f"projection=orthographic, vertical_span={orthographic_height:.2f} m",
        flush=True,
    )


def _model_generation_with_overhead_camera(*args, **kwargs):
    model, xml, objects_info = _original_model_generation_wizard(*args, **kwargs)
    _configure_overhead_camera(model)
    return model, xml, objects_info


def main():
    driver.model_generation_wizard = _model_generation_with_overhead_camera

    original_topic_name = driver.get_camera_topic_name
    original_info_topic_name = driver.get_camera_info_topic_name

    def camera_topic_name(camera):
        if camera == StretchCameras.cam_nav_rgb:
            return "/overhead_camera/image_raw"
        return original_topic_name(camera)

    def camera_info_topic_name(camera):
        if camera == StretchCameras.cam_nav_rgb:
            return "/overhead_camera/camera_info"
        return original_info_topic_name(camera)

    driver.get_camera_topic_name = camera_topic_name
    driver.get_camera_info_topic_name = camera_info_topic_name
    driver.main()


if __name__ == "__main__":
    main()
