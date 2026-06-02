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
python -m pip install --upgrade "setuptools==69.5.1" wheel
mv src/stretch_ros2/stretch_deep_perception/pyproject.toml src/stretch_ros2/stretch_deep_perception/pyproject.toml.disabled
mv src/stretch_ros2/stretch_funmap/pyproject.toml src/stretch_ros2/stretch_funmap/pyproject.toml.disabled
```

3. Build

```
colcon build --symlink-install
```
