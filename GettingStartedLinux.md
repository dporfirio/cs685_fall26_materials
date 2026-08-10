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
cd ~/ament_ws/src
git clone -b humble git@github.com:hello-robot/stretch_ros2.git
git clone git@github.com:dporfirio/cs685_fall26_materials.git
```

2. Check that it builds

```
cd ~/ament_ws
colcon build --symlink-install
source install/setup.bash
```