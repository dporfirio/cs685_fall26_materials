from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_path("stretch_description")
    course_share = get_package_share_path("cs685_fall26_materials")
    calibrated_urdf = description_share / "urdf" / "stretch.urdf"
    fallback_urdf = (
        description_share
        / "urdf"
        / "stretch_description_SE3_eoa_wrist_dw3_tool_sg3.xacro"
    )
    urdf = calibrated_urdf if calibrated_urdf.is_file() else fallback_urdf
    robot_description = ParameterValue(Command(["xacro ", str(urdf)]), value_type=str)

    record_video = LaunchConfiguration("record_video")
    output_directory = LaunchConfiguration("output_directory")
    filename = LaunchConfiguration("filename")
    video_fps = LaunchConfiguration("video_fps")
    camera_margin = LaunchConfiguration("camera_margin")
    use_cameras = LaunchConfiguration("use_cameras")
    use_mujoco_viewer = LaunchConfiguration("use_mujoco_viewer")

    arguments = [
        DeclareLaunchArgument("record_video", default_value="true"),
        DeclareLaunchArgument(
            "output_directory", default_value=str(Path.home() / "ament_ws" / "videos")
        ),
        DeclareLaunchArgument("filename", default_value=""),
        DeclareLaunchArgument("video_fps", default_value="10.0"),
        DeclareLaunchArgument("camera_margin", default_value="1.0"),
        DeclareLaunchArgument("use_cameras", default_value="true"),
        DeclareLaunchArgument("use_mujoco_viewer", default_value="false"),
        DeclareLaunchArgument("robocasa_task", default_value="PnPCounterToCab"),
        DeclareLaunchArgument("robocasa_layout", default_value="9"),
        DeclareLaunchArgument("robocasa_style", default_value="11"),
    ]

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="log",
        parameters=[
            {"source_list": ["/stretch/joint_states"]},
            {"rate": 30.0},
            {"robot_description": robot_description},
        ],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"publish_frequency": 30.0},
        ],
    )
    mujoco_driver = Node(
        package="cs685_fall26_materials",
        executable="topdown_mujoco_driver.py",
        output="screen",
        emulate_tty=True,
        additional_env={"TOPDOWN_CAMERA_MARGIN": camera_margin},
        remappings=[
            ("cmd_vel", "/stretch/cmd_vel_guarded"),
            ("joint_states", "/stretch/joint_states"),
        ],
        parameters=[
            {
                "rate": 30.0,
                "timeout": 0.5,
                "broadcast_odom_tf": True,
                "fail_out_of_range_goal": False,
                "mode": "navigation",
                "use_mujoco_viewer": ParameterValue(
                    use_mujoco_viewer, value_type=bool
                ),
                "use_cameras": ParameterValue(use_cameras, value_type=bool),
                "use_robocasa": True,
                "robocasa_task": LaunchConfiguration("robocasa_task"),
                "robocasa_layout": LaunchConfiguration("robocasa_layout"),
                "robocasa_style": LaunchConfiguration("robocasa_style"),
            }
        ],
    )
    locomotion_posture_guard = Node(
        package="cs685_fall26_materials",
        executable="locomotion_posture_guard.py",
        output="screen",
        parameters=[
            str(course_share / "config" / "locomotion_posture_guard.yaml"),
            {"use_sim_time": True},
        ],
    )
    recorder = Node(
        package="cs685_fall26_materials",
        executable="topdown_video_recorder.py",
        output="screen",
        condition=IfCondition(record_video),
        parameters=[
            {
                "use_sim_time": True,
                "output_directory": output_directory,
                "filename": filename,
                "fps": ParameterValue(video_fps, value_type=float),
            }
        ],
    )

    return LaunchDescription(
        arguments
        + [
            joint_state_publisher,
            robot_state_publisher,
            mujoco_driver,
            locomotion_posture_guard,
            recorder,
        ]
    )
