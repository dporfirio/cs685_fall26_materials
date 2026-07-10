# CS 685 Fall 2026 Materials

This is a ROS 2 Humble package containing CS 685 course materials for the
Hello Robot Stretch 3.

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

See [GettingStarted.md](GettingStarted.md) for the Stretch development
environment setup.

## MuJoCo Cameras

MuJoCo cameras are disabled by default. Start the simulator with camera
publishing explicitly enabled and its built-in RViz disabled:

```bash
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py use_mujoco_viewer:=true mode:=navigation use_cameras:=true use_rviz:=false
```

On macOS, initializing all five camera renderers can take approximately 40
seconds. Wait for `stretch_mujoco_driver started` before checking topics. The
head-camera topics used by this package are:

```text
/camera/color/image_raw
/camera/depth/image_rect_raw
/camera/color/camera_info
```

MuJoCo publishes sensor data with best-effort QoS. Match that reliability when
inspecting an image from the command line:

```bash
ros2 topic echo /camera/color/image_raw --once --qos-reliability best_effort
```

In a plain `rviz2` window, add an **Image** display for
`/camera/color/image_raw` and set **Reliability Policy** to **Best Effort**. Be
aware that running `rviz2` without arguments loads `~/.rviz2/default.rviz`; an
older saved configuration may contain stale topics or incompatible QoS.

The full `stretch_sim.rviz` profile can abort with a mutex error on the macOS
OpenGL 2.1 implementation. The smaller object-recognition profile is stable:

```bash
rviz2 -d ~/ament_ws/install/stretch_deep_perception/share/stretch_deep_perception/rviz/object_detection.rviz
```

For the complete object-recognition setup, topic remapping, and dependency
instructions, see [GettingStarted.md](GettingStarted.md#object-recognition).
