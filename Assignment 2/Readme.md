# CSCI (ARTI) 4530/6530 Introduction to Robotics - Fall 2021
## Assignment 2: Navigation of Robot to a Goal Location with Obstacles Avoidance

### Author: Ehsan Latif

#### Date: 18th October, 2021

This purpose of this assignment is to get Navigate a mobile robot to a goal location without hitting obstacles.

### About Repository
* It contains 2 ROS Packages:
  1. motion_planner_n_obstalce_avoidance

> Note: Before running the nodes Make Sure that you have already running gazeebo node by the command:
> 
> ``$ roslaunch husky_gazebo husky_playpen.launch ``
> 
> Instructions to install gazebo are available at (https://github.com/husky/husky.git)



#### motion_planner_n_obstalce_avoidance
This package contains two node files:
##### 1. motion_planner_without_obstacle.py_
A ROS node which subscribes to /odometry/filtered topic to get position information of the robot and based on the current possition it decides about the motion of the robot. The node calculates the angular velocity such that it orient robot towards the goal location. This method based on the [feedback contoller](https://github.com/SMARTlab-Purdue/ros-tutorial-gazebo-simulation/wiki/Sec.-2:-Driving-the-Husky-robot-in-Gazebo)


To run the node:

`` $ rosrun motion_planner_n_obstacle_avoider motion_planner_without_obstacle.py ``

##### 2. motion_planner_with_obstacle_avoidance.py_
A ROS node which subscribe to \laserscan topic and based on the observations it divides the laser scan data into 12 regions as mentioned [here](https://github.com/Rad-hi/Obstacle-Avoidance-ROS). For each regions it calculates the distance and find probablity of having obstacle. If there is an obstalce detected in front of the robot, it then look for clearance region. To find the clearance region, it looks for all regions and finds the one with no or far-away obstacle. Further to reach towards the goal, it finds weight for each region based on the currennt orientation of the robot. The region with the less difference  of orientation to the goal position has the high weight. Later based on the cheapest and high weighted region, applied otimization and find the one which has clear ay and orients towards goal. Then applied angular velocity on the robot based on the optimal region. Then it publiishes linear and angular velocity to the gazebo node(simulator robot) so that it turns when confronting an obstacle to the appropriate direction.
To run the node:

`` $ roslaunch motion_planner_n_obstacle_avoider launcher.launch ``
> *Note: This launches the node to perform both motion planing and obstacle avoidance simultaneusly, so there is no need to run other node for this.*


**Thank You**
---



