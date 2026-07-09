from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    stretch_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("stretch_nav2"),
                    "launch",
                    "online_async_launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    # Start only the detector here. The package's demo launch file also starts
    # the physical robot driver and RealSense camera, which require stretch_body
    # and conflict with an existing robot or simulation bringup.
    stretch_object_detector = Node(
        package="stretch_deep_perception",
        executable="detect_objects",
        output="screen",
    )

    return LaunchDescription(
        [
            stretch_nav2_launch,
            stretch_object_detector,
        ]
    )
