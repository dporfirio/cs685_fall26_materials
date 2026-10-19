"""Launch Stretch in a fixed two-room scene without layout selection prompts."""

import sys

import launch_ros.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


class _TwoRoomsLaunchDescriptionSource(PythonLaunchDescriptionSource):
    """Retain Stretch's ROS setup while selecting the course scene driver."""

    def _get_launch_description(self, location):
        original_node = launch_ros.actions.Node

        def two_rooms_node(*args, **kwargs):
            if kwargs.get("executable") == "stretch_mujoco_driver":
                kwargs["package"] = "cs685_fall26_materials"
                kwargs["executable"] = "two_rooms_mujoco_driver.py"
            return original_node(*args, **kwargs)

        # The upstream launch checks sys.argv before resolving launch arguments.
        sys.argv.append("use_robocasa:=false")
        launch_ros.actions.Node = two_rooms_node
        try:
            return super()._get_launch_description(location)
        finally:
            launch_ros.actions.Node = original_node
            sys.argv.pop()


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            _TwoRoomsLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("stretch_simulation"),
                    "launch",
                    "stretch_mujoco_driver.launch.py",
                ])
            ),
            launch_arguments={
                "use_robocasa": "false",
                "use_rviz": "false",
            }.items(),
        )
    ])
