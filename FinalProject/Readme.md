# CSCI (ARTI) 4530/6530 Introduction to Robotics - Fall 2021
## Final Exam - Project Test - Autonomous Exploration

### Author: Ehsan Latif

#### Date: 11th December, 2021

This project is the implementation of the Gazebo simulations exhibiting autonomous exploration capability of
a mobile robot (Clearpath’s Husky) fitted with a Kinect camera that provides RGB-D data and a LIDAR that
provides 2D laser scan.
The objective is to find (locate) and detect the color of the symbols throughout the building (THERE
ARE TOTALLY FIVE SYMBOLS) and come back to the Origin (0,0) once exploration is completed. The
task is simplified in a sense that there is a central server that already know where to look for symbols by
providing waypoints.

### About Repository
It contains 2 ROS Packages:
1. final_project
2. projectserver

> Note: Before running the nodes Make Sure that you have already running gazeebo and amcl navigation launchfiles by the command:
> 
> ``$ roslaunch husky_gazebo husky_playpen.launch ``
> Instructions to install gazebo are available at [repository](https://github.com/husky/husky.git)
> 
> ``$ roslaunch husky_navigation amcl_demo.launch ``
> Instructions to install amcl are available at [ros resource](http://wiki.ros.org/amcl)

#### final_project
This package contains one node files:
##### husky_mover.py_
A ROS node which subscribed to following topics:
1. */amcl_pose* topic to get robt position information of the robot for **localization**
2. */map* to fetch the map produced by amcl for **Navigation**
3. *realsense/color/image_raw* to retrieve live image feed from camera mounted on husky for **Object Detection**

The Node published the */cmd_vel* topic for robot movement.

husky_mover also connected to the *wpserver.py* to request object position and provide response when arrived and detected the requried object.

##### Path Planning (A* Path Planner)
1. Once the node received the goal coordinated from the server it then required to plan a path for movement.
2. We already have [OccupancyGrid](http://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html) which has infomration about map, object and obstacles.
3. I have implemented A* algorithm with the help of this open source [repository](https://www.programcreek.com/python/?project_name=LetsPlayNow%2FTrajectoryPlanner#) to convert Grid points to world frame coordinates and vise versa, it also provided base for detetcing obstacle using grid information.
4. The Path Planner class, receives start, destiantiona and grid, then  computed path using standard A* algorithm with the heuristic of euclidean distance from particular psition to destination.
5. After path finding, the planner calls traveler node which plans motion (linear, angular velocity) based on the path coordinates.
#### Object Detection
1. Once the robot arrived at the required position, robot fetched image feed and detect the object with requried color using openCV Contouring library.
2. If the robot able to detect the color accuratly, it then request *wpserver* to get next destination.

First run the projectserver package:

`` $ rosrun projectserver wpserver.py ``

Then run the final_project package:

`` $ roslaunch final_project launcher.launch ``

> *Note: This launches the node to perform both motion planing and object detection simultaneusly, so there is no need to run morethan one node.*

### Video
There is another folder named *video* which contains video demonstration of the project.

**Thank You**
---



