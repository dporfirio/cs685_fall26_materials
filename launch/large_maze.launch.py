"""Launch the maze with SLAM, or saved-map localization and navigation.

Run teleop_twist_keyboard in a separate terminal, remapping cmd_vel to
/stretch/cmd_vel, to explore and build the map.
"""

import sys
from pathlib import Path

import launch_ros.actions
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def _mapping_or_navigation(context):
    map_argument = LaunchConfiguration("map").perform(context).strip()
    if not map_argument:
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("stretch_nav2"), "launch", "online_async_launch.py",
            ])),
            launch_arguments={"use_sim_time": "true"}.items(),
        )]

    map_path = Path(map_argument).expanduser().resolve()
    if not map_path.is_file():
        raise FileNotFoundError(f"Map YAML file not found: {map_path}")

    # Stretch's default AMCL topic is /scan; the simulator publishes /scan_filtered.
    params = RewrittenYaml(
        source_file=PathJoinSubstitution([
            FindPackageShare("stretch_nav2"), "config", "nav2_params.yaml",
        ]),
        root_key="",
        param_rewrites={"scan_topic": "/scan_filtered"},
        convert_types=True,
    )
    return [GroupAction(scoped=True, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("nav2_bringup"), "launch", "localization_launch.py",
            ])),
            launch_arguments={
                "map": str(map_path), "use_sim_time": "true", "autostart": "true",
                "params_file": params, "use_composition": "False",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("stretch_nav2"), "launch", "navigation_launch.py",
            ])),
            launch_arguments={
                "use_sim_time": "true", "autostart": "true", "params_file": params,
            }.items(),
        ),
        LogInfo(msg=f"Navigating with {map_path}; set the initial pose in RViz."),
    ])]


class _LargeMazeLaunchDescriptionSource(PythonLaunchDescriptionSource):
    """Keep upstream sensor/TF setup while selecting the maze scene driver."""

    def _get_launch_description(self, location):
        original_node = launch_ros.actions.Node

        def maze_node(*args, **kwargs):
            if kwargs.get("executable") == "stretch_mujoco_driver":
                kwargs["package"] = "cs685_fall26_materials"
                kwargs["executable"] = "large_maze_mujoco_driver.py"
            return original_node(*args, **kwargs)

        # Upstream checks sys.argv before resolving the launch arguments.
        sys.argv.append("use_robocasa:=false")
        launch_ros.actions.Node = maze_node
        try:
            return super()._get_launch_description(location)
        finally:
            launch_ros.actions.Node = original_node
            sys.argv.pop()


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "map", default_value="",
            description="Saved map YAML path. Leave empty to build a new map with SLAM.",
        ),
        DeclareLaunchArgument("use_rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument(
            "use_mujoco_viewer", default_value="true", choices=["true", "false"]
        ),
        OpaqueFunction(function=_mapping_or_navigation),
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            _LargeMazeLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("stretch_simulation"),
                    "launch", "stretch_mujoco_driver.launch.py",
                ])
            ),
            launch_arguments={
                "use_robocasa": "false",
                "mode": "navigation",
                "broadcast_odom_tf": "True",
                "use_rviz": "false",
                "use_mujoco_viewer": LaunchConfiguration("use_mujoco_viewer"),
            }.items(),
        )]),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution([
                FindPackageShare("cs685_fall26_materials"), "rviz", "maze_mapping.rviz",
            ])],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            output="screen",
        ),
        LogInfo(msg=(
            "Drive from another sourced terminal: ros2 run teleop_twist_keyboard "
            "teleop_twist_keyboard --ros-args -r cmd_vel:=/stretch/cmd_vel "
            "-p speed:=0.3 -p turn:=0.6"
        )),
    ])
