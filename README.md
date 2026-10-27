# CS 685 Fall 2026 Materials

This is a ROS 2 Humble package containing CS 685 course materials for the
Hello Robot Stretch 3.

## Install

You must have Ubuntu 22.04, Windows 11, or MacOS (silicon) to proceed. You should not need to resort to a virtual machine.

If you are on Mac, do this: [GettingStartedMac.md](GettingStartedMac.md).
If you are on Windows, do this: [GettingStartedWindows.md](GettingStartedWindows.md).
If you are on Linux, do this: [GettingStartedLinux.md](GettingStartedLinux.md).

## Build

The package is expected to live alongside `stretch_ros2` in the workspace
source directory:

```text
~/ament_ws/
└── src/
    ├── cs685_fall26_materials/
    └── stretch_ros2/
```

From the workspace root, activate or source the ROS 2 Humble environment and
build:

```bash
cd ~/ament_ws
colcon build --symlink-install
source install/local_setup.zsh
```

To build only this package and any required workspace dependencies:

```bash
colcon build --symlink-install --packages-up-to cs685_fall26_materials
```

Launch a minimal scene with two empty 4 × 4 m rooms connected by a 2 m long,
1.5 m wide hallway (no layout selection prompts):

```bash
ros2 launch cs685_fall26_materials two_rooms.launch.py
```

The MuJoCo viewer opens by default; RViz is disabled. Standard Stretch launch
options such as `use_cameras:=true` and `mode:=navigation` are also available.

See [GettingStarted.md](GettingStarted.md) for the Stretch development
environment setup.

To drive through the large maze while SLAM builds a map, rebuild and source the
workspace as above, then run:

```bash
ros2 launch cs685_fall26_materials large_maze.launch.py
```

This starts Stretch in navigation mode, the MuJoCo viewer, SLAM Toolbox, and
RViz showing the live `/map` and laser scan. Set `use_rviz:=false` or
`use_mujoco_viewer:=false` to disable either window.

In a second terminal with the same ROS environment and workspace sourced, run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/stretch/cmd_vel -p speed:=0.3 -p turn:=0.6
```

Keep that terminal focused. Use `i` to drive forward, `j`/`l` to turn,
`,` to reverse, and `k` to stop. Explore the rooms and hallways to extend the
map; revisiting areas helps SLAM close loops. To save the current map, run in
another sourced terminal (files are written in the current directory):

```bash
ros2 run nav2_map_server map_saver_cli -f maze_map --ros-args -p use_sim_time:=true
```

To load a saved map and start AMCL localization and Nav2 navigation without SLAM:

```bash
ros2 launch cs685_fall26_materials large_maze.launch.py map:=/absolute/path/to/maze_map.yaml
```

In RViz, use **2D Pose Estimate** to set the robot's initial position and heading
on the saved map, then **Nav2 Goal** to select a destination. Keyboard driving
remains available. Omitting `map` (or passing `map:=''`) keeps SLAM mode.

Keep the **Navigation 2** panel loaded: it sends the Nav2 Goal tool's selections
to the navigation action server. If using a custom RViz configuration, add it
through **Panels → Add New Panel → nav2_rviz_plugins/Navigation 2**. The message
`Managed nodes are active` means Nav2 has finished starting and is ready for goals.

The **AMCL Pose Hypotheses** RViz display shows green arrows for AMCL's particles
on `/particle_cloud` when running with a saved map. Set the initial pose first;
each arrow represents a possible robot position and heading. This display has
no particles in SLAM mode. In an already running RViz, add it with **Add → By
display type → nav2_rviz_plugins → ParticleCloud** and select `/particle_cloud`.
