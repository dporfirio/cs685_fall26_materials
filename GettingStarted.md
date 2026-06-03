# Installing Stretch on your Own Computer

## Mac

These instructions assume that you have the latest Mac OS.

The following instructions are modified from [here](https://robostack.github.io/GettingStarted.html).

1. Download Stretch packages:

```
cd ~`
mkdir -p ament_ws & cd ament_ws/src
git clone -b humble git@github.com:hello-robot/stretch_ros2.git
```

2. Set up a micromamba environment

```
micromamba create -n stretch_humble -c conda-forge -c robostack-staging python=3.10 ros-humble-desktop compilers cmake pkg-config make ninja colcon-common-extensions
micromamba activate stretch_humble
micromamba install -c conda-forge -c robostack-staging colcon-common-extensions colcon-core colcon-ros colcon-python-setup-py ros-dev-tools
micromamba install -n stretch_humble robostack-staging::ros-humble-xacro
micromamba install -n stretch_humble -c robostack-staging -c conda-forge ros-humble-joint-state-publisher
micromamba install -n stretch_humble -c robostack-staging -c conda-forge ros-humble-control-msgs
python -m pip install --upgrade "setuptools==69.5.1" wheel
python -m pip install pyquaternion
python -m pip install transforms3d
python -m pip install git+https://github.com/DLu/tf_transformations.git
mv src/stretch_ros2/stretch_deep_perception/pyproject.toml src/stretch_ros2/stretch_deep_perception/pyproject.toml.disabled
mv src/stretch_ros2/stretch_funmap/pyproject.toml src/stretch_ros2/stretch_funmap/pyproject.toml.disabled
ln -s "$CONDA_PREFIX/bin/mjpython" "$CONDA_PREFIX/bin/mjpython.10"
```

4. Rosdep doesn't seem to work well on Mac. Better install all dependencies manually.

```
git clone -b humble git@github.com:SteveMacenski/slam_toolbox.git
```

3. Build

```
colcon build --symlink-install
```

4. Install Stretch Mujoco

```
cd ~
mkdir bin & cd bin
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
