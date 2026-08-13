from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("cs685_fall26_materials")

    forwarded_arguments = {
        "record_video": LaunchConfiguration("record_video"),
        "output_directory": LaunchConfiguration("output_directory"),
        "filename": LaunchConfiguration("filename"),
        "video_fps": LaunchConfiguration("video_fps"),
        "camera_margin": LaunchConfiguration("camera_margin"),
        "robocasa_task": LaunchConfiguration("robocasa_task"),
        "robocasa_layout": LaunchConfiguration("robocasa_layout"),
        "robocasa_style": LaunchConfiguration("robocasa_style"),
    }

    arguments = [
        DeclareLaunchArgument("record_video", default_value="true"),
        DeclareLaunchArgument(
            "output_directory",
            default_value=str(Path.home() / "ament_ws" / "videos"),
        ),
        DeclareLaunchArgument("filename", default_value=""),
        DeclareLaunchArgument("video_fps", default_value="10.0"),
        DeclareLaunchArgument("camera_margin", default_value="1.0"),
        DeclareLaunchArgument("robocasa_task", default_value="PnPCounterToCab"),
        DeclareLaunchArgument("robocasa_layout", default_value="9"),
        DeclareLaunchArgument("robocasa_style", default_value="11"),
    ]

    topdown_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [package_share, "launch", "topdown_recording_sim.launch.py"]
            )
        ),
        launch_arguments=forwarded_arguments.items(),
    )

    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "robot_bringup.launch.py"])
        )
    )

    kitchen_item_traveler = Node(
        package="cs685_fall26_materials",
        executable="kitchen_item_traveler.py",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        arguments + [topdown_simulation, robot_bringup, kitchen_item_traveler]
    )
