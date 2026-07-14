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

## Autonomous Exploration

`robot_bringup.launch.py` starts Stretch's asynchronous SLAM mapper (using the
MuJoCo laser topic `/scan_filtered`), Nav2, and an autonomous frontier explorer
in an idle state. The explorer uses the live SLAM occupancy grid and Nav2's `navigate_to_pose` action
to seek reachable free cells bordering unknown space. Candidate frontiers are
ranked by estimated unknown area within sensor range, with a penalty for travel
distance. Failed goals are blacklisted so the robot does not repeatedly try the
same unreachable location.

Start exploration with:

```bash
ros2 service call /exploration/start std_srvs/srv/Trigger "{}"
```

Stop it manually with:

```bash
ros2 service call /exploration/stop std_srvs/srv/Trigger "{}"
```

Query its state with:

```bash
ros2 service call /exploration/status std_srvs/srv/Trigger "{}"
ros2 topic echo /exploration/status_text --qos-durability transient_local
```

Exploration completes after several consecutive planning cycles find no
frontier above the configured information-gain threshold. It also stops when
several consecutive navigation goals reveal less than the configured minimum
new map area. These criteria measure expected and observed information gain;
they do not assume that explored space is enclosed by walls.

The thresholds are configured in
[`config/autonomous_explorer.yaml`](config/autonomous_explorer.yaml). Useful
parameters include:

- `minimum_information_gain`: minimum estimated unknown area, in square meters,
  required to pursue a frontier.
- `probe_unknown_space`: after normal frontiers are exhausted, search safe floor
  positions for viewpoints into smaller or isolated blank regions.
- `minimum_probe_information_gain`: lower information threshold used for those
  blank-space viewpoints.
- `minimum_progress_area`: minimum newly observed area expected from a completed
  goal.
- `low_gain_confirmation_cycles`: number of low-gain planning cycles required
  before declaring exploration complete.
- `no_progress_goal_limit`: number of consecutive successfully reached goals
  that reveal too little new map area before stopping. Navigation rejections and
  aborted paths do not count toward this limit; they blacklist that candidate
  and trigger replanning.
- `goal_timeout`: maximum navigation time for one frontier goal.
- `obstacle_clearance` / `unknown_clearance`: required free-space margins around
  a navigation goal.
- `goal_search_radius`: how far inward from a frontier the explorer may search
  for a connected, safely reachable staging point.

While navigating, the explorer checks the active goal against each updated SLAM
map. If newly observed obstacles or unknown space make it unsafe or disconnect
it from the robot's reachable free-space region, the goal is canceled,
blacklisted, and replaced with a newly planned frontier goal.
