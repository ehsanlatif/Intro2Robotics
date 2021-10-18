# CSCI (ARTI) 4530/6530 Introduction to Robotics - Fall 2021
## Assignment 1: Wheeled Kinematics and Basics of ROS

### Author: Ehsan Latif

#### Date: 24th September, 2021

This purpose of this assignment is to get familiar with the basics of robotics, wheeled kinematics, and to have a
practical understanding of Robot Operating System (ROS) based software development of robotic solutions.

### About Repository
* It contains 2 ROS Packages:
  1. husky_ehsan
  2. husky_grad_ehsan

> Note: Before running the nodes Make Sure that you have already running gazeebo node by the command:
> 
> ``$ roslaunch husky_gazebo husky_playpen.launch ``
> 
> Instructions to install gazebo are available at (https://github.com/husky/husky.git)



#### husky_ehsan
This package contains two node files:
##### 1. _move_husky.py_
A publisher node that makes the simulated Husky robot in Gazebo move with a linear speed of 0.2 m/s.

To run the node:

`` $ rosrun husky_ehsan move_husky.py ``
##### 2. _listen_husky.py_
A subscriber node that listens to the Husky’s velocity commands on the topic /cmd_vel, and converts the linear and angular velocity to the wheel velocities of an arbitrary differential drive robot using the inverse kinematics.

To run the node:

`` $ rosrun husky_ehsan move_husky.py ``
##### Input Velocity
Twist.linear.x = 0.2
##### Output Wheel Velocities
```
left_wheel_velocity: 4.0
right_wheel_velocity: 4.0
---
left_wheel_velocity: 4.0
right_wheel_velocity: 4.0
---
left_wheel_velocity: 4.0
right_wheel_velocity: 4.0
---
left_wheel_velocity: 4.0
right_wheel_velocity: 4.0
---
left_wheel_velocity: 4.0
right_wheel_velocity: 4.0
---
left_wheel_velocity: 4.0
right_wheel_velocity: 4.0
---
left_wheel_velocity: 4.0
right_wheel_velocity: 4.0
---
left_wheel_velocity: 4.0
right_wheel_velocity: 4.0
---
left_wheel_velocity: 4.0
right_wheel_velocity: 4.0
```

#### husky_grad_ehsan
This package contains two node files:
##### 1. _avoid_obstacle.py_
A ROS node which subsribe to \laserscan topic and based on the observations it publiishes angular velocity to the gazebo node(simulator robot) so that it turns when confronting a wall.
To run the node:

`` $ rosrun husky_ehsan avoid_obstacle.py ``
> *Note: This node is doing both motion and obstacle avoidance simultaneusly, so there is no need to run move_huky.py node for this.*

**Thank You**
---



