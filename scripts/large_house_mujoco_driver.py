#!/usr/bin/env python3

"""ROS Stretch driver for the packaged large-house MuJoCo scene."""

import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import mujoco
import robocasa
import robosuite
import stretch_mujoco
import stretch_mujoco_driver.stretch_mujoco_driver as driver


PACKAGE_NAME = "cs685_fall26_materials"


def _package_directory(module) -> Path:
    return Path(module.__file__).resolve().parent


def _resolve_asset_path(path: str) -> str:
    """Translate paths embedded by the model exporter to this installation."""
    mappings = (
        ("/third_party/robocasa/robocasa/", _package_directory(robocasa)),
        ("/third_party/robosuite/robosuite/", _package_directory(robosuite)),
        ("/stretch_mujoco/stretch_mujoco/", _package_directory(stretch_mujoco)),
    )
    for marker, package_directory in mappings:
        if marker in path:
            relative_path = path.split(marker, 1)[1]
            return str(package_directory / relative_path)
    return path


def _apply_start_lift(model):
    """Override the lift only when requested by the launching process."""
    fraction = os.environ.get("CS685_START_LIFT_FRACTION")
    if fraction is None:
        return
    fraction = float(fraction)
    if not math.isfinite(fraction) or not 0.0 <= fraction <= 1.0:
        raise ValueError("start_lift_fraction must be a finite number between 0 and 1")
    lift = model.joint("joint_lift")
    lower, upper = lift.range
    position = lower + fraction * (upper - lower)
    qpos_address = int(lift.qposadr[0])
    # For a slide joint, qpos0 is also its kinematic reference. Compensate
    # the body offset so changing the initial coordinate preserves the
    # physical meaning of every lift position (including its limits).
    body_id = int(lift.bodyid[0])
    axis_in_parent = lift.axis.copy()
    mujoco.mju_rotVecQuat(axis_in_parent, lift.axis, model.body_quat[body_id])
    delta = position - model.qpos0[qpos_address]
    for axis in range(3):
        model.body_pos[body_id, axis] += axis_in_parent[axis] * delta
    model.qpos0[qpos_address] = position
    # Simulator.start() calls home(); retain the requested lift target there.
    actuator_id = int(model.actuator("lift").id)
    model.keyframe("home").ctrl[actuator_id] = position


def _load_large_house_model():
    scene_path = (
        Path(get_package_share_directory(PACKAGE_NAME))
        / "casas"
        / "large_house"
        / "large_house.xml"
    )
    root = ET.parse(scene_path).getroot()
    for element in root.iter():
        asset_path = element.get("file")
        if asset_path:
            element.set("file", _resolve_asset_path(asset_path))

    xml = ET.tostring(root, encoding="unicode")
    model = mujoco.MjModel.from_xml_string(xml)
    # The included robot XML carries a pose beside the kitchen cabinets.
    # Default to the kitchen floor center and preserve the original height.
    # Launch overrides arrive through this driver process's environment.
    base = model.body("base_link")
    joint_id = int(base.jntadr[0])
    if joint_id < 0 or model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_FREE:
        raise ValueError("Expected base_link to have a free joint")
    qpos_address = int(model.jnt_qposadr[joint_id])
    start_x = float(os.environ.get("CS685_START_X", "3.04"))
    start_y = float(os.environ.get("CS685_START_Y", "-3.0"))
    start_yaw = os.environ.get("CS685_START_YAW", "model")
    if not all(math.isfinite(value) for value in (start_x, start_y)):
        raise ValueError("start_x and start_y must be finite numbers")
    model.qpos0[qpos_address : qpos_address + 2] = [start_x, start_y]
    if start_yaw != "model":
        yaw = float(start_yaw)
        if not math.isfinite(yaw):
            raise ValueError("start_yaw must be a finite angle in radians or 'model'")
        # Replace yaw while retaining the model's roll and pitch.
        quaternion = model.qpos0[qpos_address + 3 : qpos_address + 7]
        w, x, y, z = quaternion
        original_yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        half_delta = (yaw - original_yaw) / 2
        c, s = math.cos(half_delta), math.sin(half_delta)
        quaternion[:] = [c * w - s * z, c * x - s * y, c * y + s * x, c * z + s * w]
    _apply_start_lift(model)
    return model, xml, {}


def main():
    driver.model_generation_wizard = lambda **_: _load_large_house_model()
    driver.main()


if __name__ == "__main__":
    main()
