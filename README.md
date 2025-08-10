# 🦖 Bipedal Robot Project
ROS2 project for the control of a bipedal robot.

## Technologies
- <img width="80" height="40" alt="ros2_logo" src="https://github.com/user-attachments/assets/7c32ea58-4d4f-4baf-9b2c-dd4edcd72440" />: ROS2 Humble
- <img width="70" height="25" alt="micro-ROS_logo" src="https://github.com/user-attachments/assets/7e4392a2-e5dc-40ea-a72a-6e17328c33f3" />: micro-ROS
- <img width="40" height="40" alt="gazebo_logo" src="https://github.com/user-attachments/assets/14937d0f-e44d-4f3b-aff2-68ac8f7ec9c9" />: Gazebo
- <img width="40" height="40" alt="html" src="https://github.com/user-attachments/assets/2ca6fb9b-a955-41cc-bc0c-2500040ebaff" />: HTML

## Languages
C++



## How to Build

```bash

```

## How to Run

```bash

```

## Contains
* <mark>**bipedal_robot_msgs**</mark>:
  * <mark>**msg**</mark>:
    * `FootPositions.msg`: custom message for left and right foot position
    * `RobotCommands.msg`: custom message for joypad commands
  * `CMakeList.txt`: [TO COMPLETE]
  * `package.xml`: [TO COMPLETE]
* <mark>**config**</mark>:
  * `robot_parameters.yaml`: contains all the robot parameters, for fast edit
* <mark>**include**</mark>: 
  * `joystick_teleop_node.hpp`: header file for `joystick_teleop_node.cpp` in <mark>**src**</mark> folder (in the same way the next .hpp files...)
  * `state_machine_node.hpp`
  * `robot_ctrl_node.hpp`
  * `foot_trajectory_node.hpp`
  * `ik_node.hpp`
  * `robot_esp32_node.hpp`
  * `gazebo_node.hpp`
* <mark>**launch**</mark>:
  * `robot.launch.py`: launch file that allows to automatize the startup and configuration of multiple ROS2 nodes simultaneously with a unique command
* <mark>**src**</mark>:
  * `joystick_teleop_node.cpp`:
  * `state_machine_node.cpp`:
  * `robot_ctrl_node.cpp`:
  * `foot_trajectory_node.cpp`:
  * `ik_node.cpp`:
  * `robot_esp32_node.cpp`:
  * `gazebo_node.cpp`:
 
