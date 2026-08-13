from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Item travel separates positional navigation from pointing. Let a normal
    # NavigateToPose goal finish at the requested x/y regardless of heading;
    # the traveler owns the subsequent TF-controlled turn through guarded
    # cmd_vel commands.
    navigation_params = RewrittenYaml(
        source_file=PathJoinSubstitution(
            [FindPackageShare("stretch_nav2"), "config", "nav2_params.yaml"]
        ),
        root_key="",
        param_rewrites={
            "xy_goal_tolerance": "0.10",
            "yaw_goal_tolerance": "3.141592653589793",
        },
        convert_types=True,
    )

    # Use Stretch's mapper explicitly. Nav2's generic SLAM launch defaults to
    # the `scan` topic, whereas Stretch/MuJoCo publishes the filtered laser on
    # `/scan_filtered`.
    stretch_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("stretch_nav2"),
                    "launch",
                    "online_async_launch.py",
                ]
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # Start Nav2 without its localization/SLAM wrapper. The async mapper above
    # owns map -> odom and publishes the live occupancy grid used by Nav2 and
    # the autonomous explorer.
    stretch_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("stretch_nav2"), "launch", "navigation_launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "autostart": "true",
            "params_file": navigation_params,
        }.items(),
    )

    autonomous_explorer = Node(
        package="cs685_fall26_materials",
        executable="autonomous_explorer.py",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("cs685_fall26_materials"),
                    "config",
                    "autonomous_explorer.yaml",
                ]
            ),
            {"use_sim_time": True},
        ],
    )

    # Start only the detector here. The package's demo launch file also starts
    # the physical robot driver and RealSense camera, which require stretch_body
    # and conflict with an existing robot or simulation bringup.
    stretch_object_detector = Node(
        package="cs685_fall26_materials",
        executable="detect_objects_trusted.py",
        output="screen",
        remappings=[
            (
                "/camera/aligned_depth_to_color/image_raw",
                "/camera/depth/image_rect_raw",
            ),
        ],
    )

    return LaunchDescription(
        [
            stretch_slam_launch,
            stretch_navigation_launch,
            autonomous_explorer,
            stretch_object_detector,
        ]
    )
