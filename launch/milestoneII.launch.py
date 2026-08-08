from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from stretch_mujoco.robocasa_gen import get_styles, layouts


def generate_launch_description():
    stretch_mujoco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
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

    return LaunchDescription([stretch_mujoco])
