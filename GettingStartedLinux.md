# Installing Stretch on your Own Computer

## Linux

Ensure that you have Ubuntu 22.04. No other version of Linux will work for this robot.

Before cloning this repository, install ROS2 Humble: [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). When the installation branches into multiple different options, just do whatever option is "Recommended."

Make sure to add `source /opt/ros/humble/setup.bash` to the bottom of your .bashrc

```echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc```

🛑 STOP! Have you installed ROS2 in `~/ament_ws` yet? If not, go back and carefully complete the  installation instructions. 🛑

🛑 One more thing! Before moving on, ask yourself if you need to do the ROS2 tutorials. You can find them [here](https://docs.ros.org/en/humble/Tutorials.html). Doing the Python tutorials up through the "Advanced" category is recommended. 🛑

If you've done both of the above steps, then you're ready to move on...

1. Download Stretch packages. It is best practice to have placed `~/ament_ws` in your home directory, so these instructions assume that directory is your ROS2 location:

```
mkdir -p ~/ament_ws/src
cd ~/ament_ws/src
git clone -b humble git@github.com:hello-robot/stretch_ros2.git
git clone git@github.com:dporfirio/cs685_fall26_materials.git
```

2. Download dependencies

```
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

2. Check that it builds

```
cd ~/ament_ws
colcon build --symlink-install
source install/setup.bash
```

3. Install Stretch Mujoco. Note that the newer versions of stretch_mujoco are installable with uv, but we need to guide the installation to use the system's version of python that ROS2 uses:

```
cd ~
mkdir -p bin && cd bin
git clone git@github.com:hello-robot/stretch_mujoco.git
cd stretch_mujoco
git submodule update --init --recursive
curl -LsSf https://astral.sh/uv/install.sh | sh
exec $SHELL
uv sync
mkdir -p ~/.local/lib/python3.10/site-packages
uv pip install --python /usr/bin/python3 --target ~/.local/lib/python3.10/site-packages -e .

uv pip install \
  --python /usr/bin/python3 \
  --target ~/.local/lib/python3.10/site-packages \
  -e ".[robocasa]"

uv pip install \
  --python /usr/bin/python3 \
  --target ~/.local/lib/python3.10/site-packages \
  -e "robocasa @ ./third_party/robocasa"

uv pip install \
  --python /usr/bin/python3 \
  --target ~/.local/lib/python3.10/site-packages \
  -e "robosuite @ ./third_party/robosuite"

/usr/bin/python3 third_party/robosuite/robosuite/scripts/setup_macros.py
/usr/bin/python3 third_party/robocasa/robocasa/scripts/setup_macros.py
/usr/bin/python3 third_party/robocasa/robocasa/scripts/download_kitchen_assets.py
```

4. Install a few extra things:

```
sudo apt update
sudo apt install ros-humble-xacro portaudio19-dev ros-humble-joint-state-publisher
/usr/bin/python3 -m pip install --user pyquaternion
```

4. Get Stretch's URDF.

The stretch_simulation MuJoCo launch file expects the following robot description: `stretch_description_SE3_eoa_wrist_dw3_tool_sg3.xacro`. However, the `stretch_ros2/stretch_description/urdf` directory does not contain these generated SE3 files by default.

The `hello-robot-stretch-urdf` package installed with stretch_mujoco contains the required uncalibrated SE3 URDF/Xacro files. Copy them as such:

```
cp -r \ ~/.local/lib/python3.10/site-packages/stretch_urdf/SE3/xacro/* \ ~/ament_ws/src/stretch_ros2/stretch_description/urdf/ 
cp \ ~/.local/lib/python3.10/site-packages/stretch_urdf/SE3/*.urdf \ ~/ament_ws/src/stretch_ros2/stretch_description/urdf/
```

5. Test the simulator:

```
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py mode:=navigation use_cameras:=true use_rviz:=false
```
