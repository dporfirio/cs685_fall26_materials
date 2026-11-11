import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


class _LargeHouseLaunchDescriptionSource(PythonLaunchDescriptionSource):
    """Use the course driver while retaining Stretch's standard ROS setup."""

    def _get_launch_description(self, location):
        included_arguments = [
            "use_robocasa:=true",
            "robocasa_layout:=Random",
            "robocasa_style:=Random",
            "use_rviz:=false",
        ]
        original_node = launch_ros.actions.Node

        def large_house_node(*args, **kwargs):
            if kwargs.get("executable") == "stretch_mujoco_driver":
                kwargs["package"] = "cs685_fall26_materials"
                kwargs["executable"] = "large_house_mujoco_driver.py"
                environment = dict(kwargs.get("additional_env") or {})
                environment.update({
                    "CS685_START_X": LaunchConfiguration("start_x"),
                    "CS685_START_Y": LaunchConfiguration("start_y"),
                    "CS685_START_YAW": LaunchConfiguration("start_yaw"),
                })
                kwargs["additional_env"] = environment
            return original_node(*args, **kwargs)

        import sys

        sys.argv.extend(included_arguments)
        launch_ros.actions.Node = large_house_node
        try:
            return super()._get_launch_description(location)
        finally:
            launch_ros.actions.Node = original_node
            del sys.argv[-len(included_arguments) :]


def generate_launch_description():
    start_arguments = [
        DeclareLaunchArgument("start_x", default_value="3.04", description="Initial world x position in meters"),
        DeclareLaunchArgument("start_y", default_value="-3.0", description="Initial world y position in meters"),
        DeclareLaunchArgument("start_yaw", default_value="model", description="Initial world yaw in radians, or model to retain the scene orientation"),
    ]
    stretch_launch = IncludeLaunchDescription(
        _LargeHouseLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("stretch_simulation"),
                    "launch",
                    "stretch_mujoco_driver.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_robocasa": "true",
            "robocasa_layout": "Random",
            "robocasa_style": "Random",
            "use_rviz": "false",
        }.items(),
    )
    return LaunchDescription(start_arguments + [stretch_launch])
