#!/usr/bin/env python3

"""ROS Stretch driver for the packaged large-house MuJoCo scene."""

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
    return mujoco.MjModel.from_xml_string(xml), xml, {}


def main():
    driver.model_generation_wizard = lambda **_: _load_large_house_model()
    driver.main()


if __name__ == "__main__":
    main()
