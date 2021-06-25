## 1. Description

This package is used for simulating the robot in Gazebo. 

## 2. Requirements
The Gazebo software and the ROS packages for interfacing with Gazebo, called gazebo_ros_pkgs, must be installed. 


## 3. Run

To start up Gazebo, run: 

```
$ ros2 launch kmr_simulation gazebo.launch.py
```
This will launch a model of the robot in Gazebo, and it is possible to control it, by using the scripts in the kmr_navigation2 folder. 
```
For instance, we can run the twist keyboard script:
```
$ ros2 run kmr_navigation2 twist_keyboard.py
```
The keyboard will make the robot move around in the simulated environment. 
