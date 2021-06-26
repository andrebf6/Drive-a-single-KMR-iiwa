# Drive-a-single-KMR-iiwa
Repository containing software developed by Andrea Bravo Forn, as part of a final degree project in engineering physics performed at Federico II University (Naples, June 2021).

 <img align="left" width="300" height="200" src="https://user-images.githubusercontent.com/81975803/123516981-21ca9200-d69f-11eb-94cf-cb00b1ed7512.jpg">
 
The aim of this project was to design and implement in ROS a collision avoidance strategy for two KMR iiwa robots naviganting in a warehouse environment. 
To do so, the KMR iiwa robot model developed by Charlotte Heggem, Nina Marie Wahl and Morten M. Dahl (as a specialization project in Robotics & Automation at NTNU) has been used.
The developed code is tested in Gazebo.

The present repository contains a navigation algorithm to drive a single KMR iiwa robot in a warehouse environment.
In the repository andrebf6/Collision-avoidance-strategy-for-two-KMR-iiwa-robots-in-a-warehouse-crosssing, the implemented collision avoidance startegy for two KMR iiwa robots can be found.



System requirements:

 -  Ubuntu 18.04.3
 -  Python 3.6.9
 -  ROS2 Foxy Fitzroy
 -  Gazebo 11.0.0

Required ROS Packages:

  - Gazebo packages
  
  # Guide
  The repository contains the following packages:
  
   -  kmr_navigation2
      This package handles navigation of the KMR iiwa robot.
   -  kmr_simulation
      This package sets up the simulations in Gazebo. 
 
   
  
