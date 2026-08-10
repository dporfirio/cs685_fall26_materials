import sys

import launch_ros.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from stretch_mujoco.robocasa_gen import get_styles, layouts


class _RoboCasaPythonLaunchDescriptionSource(PythonLaunchDescriptionSource):
    """Load Stretch's launch without its command-line-only RoboCasa prompts."""

    def _get_launch_description(self, location):
        included_arguments = [
            "robocasa_layout:=included",
            "robocasa_style:=included",
        ]
        original_node = launch_ros.actions.Node

        def milestone_node(*args, **kwargs):
            if kwargs.get("executable") == "stretch_mujoco_driver":
                kwargs["package"] = "cs685_fall26_materials"
                kwargs["executable"] = "milestoneII_mujoco"
            return original_node(*args, **kwargs)

        sys.argv.extend(included_arguments)
        launch_ros.actions.Node = milestone_node
        try:
            return super()._get_launch_description(location)
        finally:
            launch_ros.actions.Node = original_node
            del sys.argv[-len(included_arguments) :]


def generate_launch_description():
    stretch_mujoco = IncludeLaunchDescription(
        _RoboCasaPythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("stretch_simulation"),
                    "launch",
                    "stretch_mujoco_driver.launch.py",
                ]
            )
        ),
        launch_arguments={
            "mode": "position",
            "use_rviz": "false",
            "use_robocasa": "true",
            "robocasa_layout": layouts[0],
            "robocasa_style": get_styles()[0],
        }.items(),
    )

    robot_kinematics = Node(
        package="cs685_fall26_materials",
        executable="robot_kinematics",
        output="screen",
    )
    controller = Node(
        package="cs685_fall26_materials",
        executable="milestoneII_controller",
        output="screen",
    )

    return LaunchDescription([stretch_mujoco, robot_kinematics, controller])
