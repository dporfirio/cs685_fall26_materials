# Installing Stretch on your Own Computer

These instructions assume that you have MacOS. They should work regardless of whether you have a Silicon or Intel chip.

The first thing you need to do is decide on whether you want to use a Virtual Machine (e.g., VirtualBox or UTM) or whether you want to install it natively on your mac.

| Option | Pros | Cons |
| --- | --- | --- |
| Native installation | Much faster than VirtualBox | Has a much more involved setup procedure |
| Virtual machine | You can use a clean Ubuntu environment (works best with robotics) | Is slower; Requires some extra configuration depending on which Virtual Machine platform you use; |

Personally, I recommend trying **Option 1: Native Installation**. If you really can't get that to work, move to Option 2.

## Option 1: Native Installation

Before cloning this repository, complete these RoboStack instructions: [https://robostack.github.io/GettingStarted.html](https://robostack.github.io/GettingStarted.html).

🛑 STOP! Have you installed ROS2 in `~/ament_ws` yet? If not, go back and carefully complete the  RoboStack instructions. 🛑

🛑 One more thing! Before moving on, ask yourself if you need to do the ROS2 tutorials. You can find them [here](https://docs.ros.org/en/humble/Tutorials.html). Doing the Python tutorials up through the "Advanced" category is recommended. 🛑

If you've done both of the above steps, then you're ready to move on...

1. Download Stretch packages. It is best practice to have placed `~/ament_ws` in your home directory, so these instructions assume that directory is your ROS2 location:

```
mkdir -p ~/ament_ws/src
cd ~/ament_ws/src
git clone -b humble git@github.com:hello-robot/stretch_ros2.git
```

2. Set up a micromamba environment

```
brew install micromamba
micromamba create -n stretch_humble -c conda-forge -c robostack-staging python=3.10 ros-humble-desktop compilers cmake pkg-config make ninja colcon-common-extensions
micromamba activate stretch_humble
micromamba install -c conda-forge -c robostack-staging colcon-common-extensions colcon-core colcon-ros colcon-python-setup-py ros-dev-tools ros-humble-slam-toolbox
micromamba install -n stretch_humble robostack-staging::ros-humble-xacro
micromamba install -n stretch_humble -c robostack-staging -c conda-forge ros-humble-joint-state-publisher ros-humble-control-msgs ros-humble-navigation2=1.1.5 ros-humble-nav2-bringup=1.1.6
micromamba install -n stretch_humble -c conda-forge pandas ipython
python -m pip install --upgrade "setuptools==79.0.1" wheel
python -m pip install "torch==2.13.0" "torchvision==0.28.0" requests seaborn GitPython thop
mv ~/ament_ws/src/stretch_ros2/stretch_deep_perception/pyproject.toml ~/ament_ws/src/stretch_ros2/stretch_deep_perception/pyproject.toml.disabled
mv ~/ament_ws/src/stretch_ros2/stretch_funmap/pyproject.toml ~/ament_ws/src/stretch_ros2/stretch_funmap/pyproject.toml.disabled
ln -s "$CONDA_PREFIX/bin/mjpython" "$CONDA_PREFIX/bin/mjpython.10"
```

The PyTorch and TorchVision versions above are a matched pair. The
`setuptools` version is also intentional: PyTorch requires version 77 or newer,
while `colcon-core` requires a version older than 80.

Do not install the complete YOLOv5 `requirements.txt` file. It includes a pip
build of OpenCV that conflicts with the conda OpenCV and OpenMP libraries used
by this environment.

3. Build

```
cd ~/ament_ws
colcon build --symlink-install
```

4. Install Stretch Mujoco. Note that this does not have to be done in a bin directory, but rather, it can be done anywhere:

```
cd ~
mkdir -p bin && cd bin
git clone git@github.com:hello-robot/stretch_mujoco.git
cd stretch_mujoco
git submodule update --init --recursive
python -m pip install -e third_party/robosuite
python -m pip install -e third_party/robocasa
python -m pip install -e .
python third_party/robosuite/robosuite/scripts/setup_macros.py
python third_party/robocasa/robocasa/scripts/setup_macros.py
python third_party/robocasa/robocasa/scripts/download_kitchen_assets.py
```

5. Install the Stretch URDF package

```
python -m pip install --upgrade hello-robot-stretch-urdf
git clone --depth 1 https://github.com/hello-robot/stretch_urdf.git /tmp/stretch_urdf

python /tmp/stretch_urdf/tools/stretch_urdf_ros_update.py \
  --model SE3 \
  --tool eoa_wrist_dw3_tool_sg3 \
  -y

cd ~/ament_ws
colcon build --symlink-install --packages-select stretch_description
```

6. Recommended: open this project in VSCode. Doing so will load the required terminals and
   the correct environments.

### MacOS intricies with object recognition (you can stop here if you're not dealing with vision yet)

The course bringup launch starts Stretch's YOLOv5 object detector alongside
online SLAM. It does not start the MuJoCo simulator itself. Build the course
package and source the workspace:

```
cd ~/ament_ws
colcon build --symlink-install
source install/setup.zsh
```

In the VSCode `mujoco` terminal, start the simulator with camera publishing
enabled:

```
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py mode:=navigation use_cameras:=true use_rviz:=false
```

On macOS, initializing MuJoCo's five camera renderers takes approximately 40
seconds. Wait until the terminal prints `stretch_mujoco_driver started` before
checking the camera topics. More complex scenes may take longer.

Then, in the VSCode `robot` terminal, start SLAM and object detection:

```
ros2 launch cs685_fall26_materials robot_bringup.launch.py
```

On its first run, the detector downloads the pinned YOLOv5 v7.0 source and the
pretrained `yolov5s.pt` weights into `~/.cache/torch/hub`. An internet
connection is required for this first run. Later runs use the cached files.

The detector uses the following RGB-D camera topics. The bringup launch remaps
the real Stretch aligned-depth topic expected by `stretch_deep_perception` to
MuJoCo's `/camera/depth/image_rect_raw` topic:

```
/camera/color/image_raw
/camera/depth/image_rect_raw
/camera/color/camera_info
```

Annotated images and 3D detection markers are published on topics under
`/objects`, including:

```
/objects/color/image_with_bb
/objects/marker_array
```

MuJoCo publishes camera images with best-effort sensor-data QoS. To inspect a
frame with the ROS CLI, request matching reliability:

```
ros2 topic echo /camera/color/image_raw --once --qos-reliability best_effort
```

The course detector wrapper likewise configures its synchronized RGB, depth,
and camera-info subscriptions with sensor-data QoS so they are compatible with
MuJoCo's publishers.

Start RViz with Stretch's object-detection configuration:

```
rviz2 -d ~/ament_ws/install/stretch_deep_perception/share/stretch_deep_perception/rviz/object_detection.rviz
```

This profile already includes an **Image** display for
`/objects/color/image_with_bb` and a **MarkerArray** display for
`/objects/marker_array`. The larger `stretch_sim.rviz` profile aborts with a
mutex error on the macOS OpenGL 2.1 implementation and should not be used on
this setup. If adding a display for a raw MuJoCo camera or laser topic manually,
set its **Reliability Policy** to **Best Effort**.

### Troubleshooting

**RViz: `base_link` does not exist**

Make ROS 2 discovery settings consistent in every terminal running MuJoCo, RViz, or ROS commands:

```zsh
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI
```

For ROS CLI terminals, restart the daemon:

```zsh
ros2 daemon stop
ros2 daemon start
```

Then verify that the simulator and TF are discoverable:

```zsh
ros2 topic echo /clock --once
ros2 run tf2_ros tf2_echo odom base_link
```

If TF is still unavailable, ensure the MuJoCo simulator is running with the same settings.

## Option 2: Virtual Machine

1. Install Ubuntu 22.04 in a Virtual Machine (e.g., VirtualBox or UTM).

   I tried UTM with the following settings:
   - Virtualization (rather than emulation) because it is faster. The only way you can get virtualization working on a Silicon Mac is by using the Ubuntu Server .iso for the ARM architecture. You can find that here: https://cdimage.ubuntu.com/ubuntu/releases/22.04/release/
   - GUI: Once Ubuntu Server was installed, I installed the Ubuntu MATE Gnome Desktop. 
   - Disk space: 42GB
   - Memory: 12GB
   
3. Then, follow the steps here: [GettingStartedLinux.md](GettingStartedLinux.md)
