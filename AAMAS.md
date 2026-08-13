### To build

```
cd ~/ament_ws
colcon build --symlink-install --packages-select cs685_fall26_materials
source install/setup.zsh
```

### To run

In one terminal, run: 

```
ros2 launch cs685_fall26_materials aamas.launch.py
```

In the other terminal, add your sequence of locations for the robot to visit:

```
ros2 topic pub --once /kitchen_travel/items std_msgs/msg/String \
  "{data: 'bread, plate, tomato, plate, knife, plate, bacon, plate'}"
```

Modify the locations to visit as needed. 