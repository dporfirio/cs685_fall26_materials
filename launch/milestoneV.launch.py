"""Launch milestone V in the large house environment."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_arguments = [
        DeclareLaunchArgument(
            "start_lift_fraction", default_value="0.8",
            description="Initial lift travel fraction: 0 is bottom, 1 is top",
        ),
        DeclareLaunchArgument("start_x", default_value="3.0", description="Initial world x position in meters"),
        DeclareLaunchArgument("start_y", default_value="-1.0", description="Initial world y position in meters"),
        DeclareLaunchArgument("start_yaw", default_value="3.14", description="Initial world yaw in radians, or model to retain the scene orientation"),
    ]
    return LaunchDescription(start_arguments + [
        GroupAction([
            # Use the project's recommended ROS middleware, preserving overrides.
            SetEnvironmentVariable(
                "RMW_IMPLEMENTATION",
                EnvironmentVariable("RMW_IMPLEMENTATION", default_value="rmw_cyclonedds_cpp"),
            ),
            SetEnvironmentVariable(
                "CS685_START_LIFT_FRACTION", LaunchConfiguration("start_lift_fraction")
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("cs685_fall26_materials"),
                        "launch",
                        "large_house_bringup.launch.py",
                    ])
                ),
                launch_arguments={
                    "start_x": LaunchConfiguration("start_x"),
                    "start_y": LaunchConfiguration("start_y"),
                    "start_yaw": LaunchConfiguration("start_yaw"),
                    "mode": "position",
                }.items(),
            ),
        ]),
    ])
