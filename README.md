# My Robot Project

Progetto ROS 2 in C++ per il controllo del robot.

## Requisiti

- ROS 2 Humble
- Ubuntu 22.04
- C++

## Build

```bash
cd my_robot_project
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Run

```bash
source install/setup.bash
ros2 launch my_robot_control control.launch.py
```
