## 1. Description

This package handles navigation of the KMR iiwa robot. 

## 2. Run

In a terminal source the ROS workspace and run:
```
    ros2 launch kmr_simulation gazebo.launch.py
```
To drive the robot to a desired position, in another terminal source the ROS workspace and run:
```
    ros2 run kmr_navigation2 navigate_point.py
```
Then, enter the desired position in the same terminal using the keyboard.

To make the robot follow a trajectory by giving some waypoints, in another terminal source the ROS workspace and run:
```
    ros2 run kmr_navigation2 navigate_waypoint.py
``` 
Then, enter the number of waypoints used for the navigation and the waypoints themselves.

To change the speed, make the rubut stop, turn to the left or turn to the right, in a new terminal  source the ROS workspace and run: 
In a new terminal, new commands can be sent to drive the robot around:
```
    ros2 run kmr_navigation2 keyboard.py
```
After this, follow the commands that will appear on the screen.

To change the simulation environment, open the gazebo.launch.py file, and change the variable word to the desired environment model (one of the SDF world models contained in the folder world).

To change the launched robot model, open the xml file of the launched simulation environment, and change the spawned robot model.

