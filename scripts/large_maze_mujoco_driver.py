#!/usr/bin/env python3

"""ROS Stretch driver for the large hallway and room maze."""

from minimal_mujoco_driver import MinimalStretchMujocoSimulator
import stretch_mujoco_driver.stretch_mujoco_driver as driver
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class LocalizationSafeStaticTransformBroadcaster(StaticTransformBroadcaster):
    """Leave map -> odom to AMCL/SLAM, preserving the robot's static TF links.

    The upstream simulator publishes an identity map -> odom on /tf_static
    every cycle. Mixing it with AMCL's dynamic /tf replaces TF's cache and
    causes extrapolation errors at the current scan and planning timestamps.
    """

    def sendTransform(self, transform):
        transforms = transform if isinstance(transform, (list, tuple)) else [transform]
        transforms = [
            item for item in transforms
            if not (
                item.header.frame_id.lstrip("/") == "map"
                and item.child_frame_id.lstrip("/") == "odom"
            )
        ]
        if transforms:
            super().sendTransform(transforms)


class LargeMazeStretchMujocoSimulator(MinimalStretchMujocoSimulator):
    scene_name = "large_maze.xml"


def main():
    driver.StretchMujocoSimulator = LargeMazeStretchMujocoSimulator
    driver.StaticTransformBroadcaster = LocalizationSafeStaticTransformBroadcaster
    driver.main()


if __name__ == "__main__":
    main()
