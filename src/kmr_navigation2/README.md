## 1. Description

This package handles navigation of the KMR iiwa robot. 

## 2. Requirements
The following packages needs to be installed:
- None

## 3. Run

First, open a terminal and run the following command to open Gazebo: 

```
$ ros2 launch kmr_simulation gazebo.launch.py
```
In a new terminal, new commands can be sent to drive the robot around:
```
1) Send velocity comands using a keyboard. This can be launched by running:
```
$ ros2 run kmr_navigation2 twist_keyboard.py
```
2) The robot can also be navigating by using a pose keyboard, where you are giving a desired pose of the robot. This can be launched by running: 
```
$ ros2 run kmr_navigation2 pose_keyboard.py
```

