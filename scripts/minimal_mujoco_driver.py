#!/usr/bin/env python3

"""ROS Stretch driver for the course's robot-and-floor-only MuJoCo scene."""

from pathlib import Path
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
import stretch_mujoco
from stretch_mujoco import StretchMujocoSimulator
import stretch_mujoco_driver.stretch_mujoco_driver as driver


PACKAGE_NAME = "cs685_fall26_materials"
_scene_directory = None


def _prepare_minimal_scene() -> Path:
    """Put the scene beside Stretch's XML and assets in a temporary directory."""
    global _scene_directory
    _scene_directory = tempfile.TemporaryDirectory(prefix="cs685_minimal_mujoco_")
    scene_directory = Path(_scene_directory.name)

    course_scene = (
        Path(get_package_share_directory(PACKAGE_NAME))
        / "worlds"
        / "minimal_stretch.xml"
    )
    robot_xml = Path(stretch_mujoco.default_robot_xml_path)

    shutil.copy2(course_scene, scene_directory / course_scene.name)
    shutil.copy2(robot_xml, scene_directory / "stretch.xml")
    (scene_directory / "assets").symlink_to(robot_xml.parent / "assets")
    return scene_directory / course_scene.name


class MinimalStretchMujocoSimulator(StretchMujocoSimulator):
    """Select the minimal course scene when the driver supplies no custom model."""

    def __init__(self, scene_xml_path=None, model=None, **kwargs):
        if scene_xml_path is None and model is None:
            scene_xml_path = str(_prepare_minimal_scene())
        super().__init__(scene_xml_path=scene_xml_path, model=model, **kwargs)


def main():
    driver.StretchMujocoSimulator = MinimalStretchMujocoSimulator
    driver.main()


if __name__ == "__main__":
    main()
