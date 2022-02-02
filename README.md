# Drive a single KMR iiwa

## 1. Description
Repository containing the software developed by Andrea Bravo Forn, as part of a bachelor degree thesis in Engineering Physics performed at the University of Naples Federico II (June 2021).

 <img align="left" width="300" height="200" src="https://user-images.githubusercontent.com/81975803/123516981-21ca9200-d69f-11eb-94cf-cb00b1ed7512.jpg">
 
This thesis on AGV fleed management lays within the framework of the European project ICOSAF. The aim of this project was to design, implement and test a distributed collision avoidance strategy for two KMR iiwa robots operating in a known, grid-like structured, smart warehouse. To test the developed strategy in a very realistic setting, ROS and Gazebo were used. Moreover, the KMR iiwa ROS model developed by Charlotte Heggem, Nina Marie Wahl and Morten M. Dahl (as a specialization project in Robotics & Automation at NTNU) has been used.

The present repository contains a navigation algorithm to drive a single KMR iiwa robot in a warehouse environment.
In the repository andrebf6/Collision-avoidance-strategy-for-two-KMR-iiwa-robots-in-a-warehouse-crosssing, the implemented collision avoidance startegy for two KMR iiwa robots can be found.



System requirements:

 -  Ubuntu 18.04.3
 -  Python 3.6.9
 -  ROS2 Foxy Fitzroy
 -  Gazebo 11.0.0

Required ROS Packages:

  - Gazebo packages
  
  ## 2. Guide
  The repository contains the following packages:
  
   -  kmr_navigation2
  
  This package handles the navigation of the KMR iiwa robot.
  
  -  kmr_simulation
  
  This package sets up the simulations in Gazebo. 
  
  ## 3. Run

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
 
   
  
