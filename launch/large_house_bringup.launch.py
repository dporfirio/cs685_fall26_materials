"""Initialize Stretch in the large house environment."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_arguments = [
        DeclareLaunchArgument("start_x", default_value="3.04", description="Initial world x position in meters"),
        DeclareLaunchArgument("start_y", default_value="-3.0", description="Initial world y position in meters"),
        DeclareLaunchArgument("start_yaw", default_value="model", description="Initial world yaw in radians, or model to retain the scene orientation"),
    ]
    return LaunchDescription(start_arguments + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("cs685_fall26_materials"),
                    "launch",
                    "large_house.launch.py",
                ])
            ),
            launch_arguments={
                name: LaunchConfiguration(name)
                for name in ("start_x", "start_y", "start_yaw")
            }.items(),
        ),
    ])
