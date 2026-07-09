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
