import sys

import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


class _StretchLaunchDescriptionSource(PythonLaunchDescriptionSource):
    """Load Stretch's launch without RoboCasa prompts in minimal mode."""

    def _get_launch_description(self, location):
        minimal = "minimal:=true" in sys.argv
        original_node = launch_ros.actions.Node

        def minimal_node(*args, **kwargs):
            if minimal and kwargs.get("executable") == "stretch_mujoco_driver":
                kwargs["package"] = "cs685_fall26_materials"
                kwargs["executable"] = "minimal_mujoco_driver.py"
            return original_node(*args, **kwargs)

        if minimal:
            sys.argv.append("use_robocasa:=false")
            launch_ros.actions.Node = minimal_node
        try:
            return super()._get_launch_description(location)
        finally:
            if minimal:
                launch_ros.actions.Node = original_node
                sys.argv.pop()


def generate_launch_description():
    minimal = LaunchConfiguration("minimal")

    launch_arguments = [
        DeclareLaunchArgument(
            "minimal",
            default_value="false",
            choices=["true", "false"],
            description="Load only the robot and its default plane, without RoboCasa",
        ),
        DeclareLaunchArgument(
            "broadcast_odom_tf",
            default_value="True",
            choices=["True", "False"],
        ),
        DeclareLaunchArgument(
            "fail_out_of_range_goal",
            default_value="False",
            choices=["True", "False"],
        ),
        DeclareLaunchArgument(
            "mode",
            default_value="position",
            choices=["position", "navigation", "trajectory", "gamepad"],
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="true", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "use_mujoco_viewer", default_value="true", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "use_cameras", default_value="false", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "use_robocasa", default_value="true", choices=["true", "false"]
        ),
        DeclareLaunchArgument("robocasa_task", default_value="PnPCounterToCab"),
        DeclareLaunchArgument("robocasa_layout", default_value="Random"),
        DeclareLaunchArgument("robocasa_style", default_value="Random"),
    ]

    use_robocasa = PythonExpression(
        [
            "'false' if '",
            minimal,
            "' == 'true' else '",
            LaunchConfiguration("use_robocasa"),
            "'",
        ]
    )

    stretch_launch = IncludeLaunchDescription(
        _StretchLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("stretch_simulation"),
                    "launch",
                    "stretch_mujoco_driver.launch.py",
                ]
            )
        ),
        launch_arguments={
            "broadcast_odom_tf": LaunchConfiguration("broadcast_odom_tf"),
            "fail_out_of_range_goal": LaunchConfiguration("fail_out_of_range_goal"),
            "mode": LaunchConfiguration("mode"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_mujoco_viewer": LaunchConfiguration("use_mujoco_viewer"),
            "use_cameras": LaunchConfiguration("use_cameras"),
            "use_robocasa": use_robocasa,
            "robocasa_task": LaunchConfiguration("robocasa_task"),
            "robocasa_layout": LaunchConfiguration("robocasa_layout"),
            "robocasa_style": LaunchConfiguration("robocasa_style"),
        }.items(),
    )

    return LaunchDescription([*launch_arguments, stretch_launch])
