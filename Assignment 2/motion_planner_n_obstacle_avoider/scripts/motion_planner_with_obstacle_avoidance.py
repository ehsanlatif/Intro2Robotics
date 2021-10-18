#!/usr/bin/env python3

import rospy, tf
from geometry_msgs.msg import Twist #ros msg that deals with moving the robot
from sensor_msgs.msg import LaserScan #ros msg that gets the laser scans
from nav_msgs.msg import Odometry
from math import *
import numpy as np
import time

# import time

# obstacle threshhold, objects a this distance or below it
#are considered obstacles
OBSTACLE_DIST = 1
#the angle in which each region extends
REGIONAL_ANGLE = 60
PI = 3.141592653

#when there's no obstacles, the robot will move with this linear velocity
NORMAL_LIN_VEL = 0.5 #meters/second
#after detecting an obstacle, the robot shall back up a bit (negative) while
# rotating to help in case it can't perform a stationary rotation
TRANS_LIN_VEL = -0.08
#the robot always rotates with the same value of angular velocity
TRANS_ANG_VEL = 1

GOAL_ACHEIVED = False

GOAL_ORIENTATION = 0



#this list keeps track of the order in which the regions' readings are obtained
REGIONS = [
             "front_C", "front_L", "left_R",
             "left_C", "left_L", "back_R",
             "back_C", "back_L", "right_R",
             "right_C", "right_L", "front_R",
          ]
#this is a global variable that keeps handles the orders for the robot to follow
#if there's a detected object, "act" is turned to True
#and the angular_vel and sleep values are calculated appropriately
Urgency_Report = {
                    "act": False, "angular_vel": 0.0, "sleep": 0
                 }
#this dict keeps track of the distance measures for each region
Regions_Report = {
                     "front_C":[], "front_L":[], "left_R":[],
                     "left_C":[], "left_L":[], "back_R":[],
                     "back_C":[], "back_L":[], "right_R":[],
                     "right_C":[], "right_L":[], "front_R":[],
                 }
#These are the costs to deviate from each region to the goal region (front_C)
Regions_Distances = {
                     "front_C": 0, "front_L": 1, "left_R": 2,
                     "left_C": 3, "left_L": 4, "back_R": 5,
                     "back_C": 6, "back_L": -5, "right_R": -4,
                     "right_C": -3, "right_L": -2, "front_R": -1,
                 }


#in this function the clearest paths are calculated and the appropriate
#values for the angular_vel and the execution times are assigned
def ClearanceTest():
    global Urgency_Report

    goal = "front_C"
    closest = 10e6
    regional_dist = 0
    obstacle_detected = False
    max_weight = 10e-6
    maxima = {"destination": "left_C", "distance": 10e-6}
    for region in Regions_Report.items():
        regional_dist = abs(Regions_Distances[region[0]]-Regions_Distances[goal])
        regional_orientation = (Regions_Distances[region[0]]*(REGIONAL_ANGLE/2))*(PI/180)*TRANS_ANG_VEL
        region_weight = (np.sign(regional_orientation) if np.sign(regional_orientation) !=0 else 1) * (np.sign(GOAL_ORIENTATION) if np.sign(GOAL_ORIENTATION) !=0 else 1) * abs(regional_orientation-GOAL_ORIENTATION) #) if abs(regional_orientation-goal_vel.angular.z) > 0 else 10
        #if there're no obstacles in that region
        # print("region: ",region,"weight",region_weight)
        if not len(region[1]) :
            #check if it's the cheapest option
            if (regional_dist < closest) and regional_dist >1:# and region_weight < max_weight:
                closest = regional_dist
                maxima["distance"] = OBSTACLE_DIST
                maxima["destination"] = region[0]
                max_weight = region_weight
        #check if it's the clearest option
        elif(max(region[1]) > maxima["distance"]):# and region_weight < max_weight:
            maxima["distance"] = max(region[1])
            maxima["destination"] = region[0]
            max_weight = region_weight
        if(regional_dist <=1 and len(region[1])>=1):
            obstacle_detected = True






    #calculate the cost to the chosen orientation
    regional_dist = Regions_Distances[maxima["destination"]]-Regions_Distances[goal]
    # print(regional_dist)

    #we act whenever the clearest path is not the front_C (front center)

    Urgency_Report["act"] = obstacle_detected
    Urgency_Report["angular_vel"] = (regional_dist*(REGIONAL_ANGLE/2))*(PI/180)*TRANS_ANG_VEL
    Urgency_Report["sleep"] = (regional_dist*(REGIONAL_ANGLE/2))*(PI
                              /180)

def IdentifyRegions(scan):
    global Regions_Report

    for region in Regions_Distances:
        i = Regions_Distances[region]
        if i < 6 :
            Regions_Report[region] = [x for x in scan.ranges[330-REGIONAL_ANGLE*i : 330-REGIONAL_ANGLE*(i-1)-1] if x <= OBSTACLE_DIST and x != 'inf']
        else:
            Regions_Report[region] = [x for x in scan.ranges[690 : 700] if x <= OBSTACLE_DIST and x != 'inf']
            Regions_Report[region].extend(x for x in scan.ranges[15 : 29] if x <= OBSTACLE_DIST and x != 'inf')
        
    # print(Regions_Report)
    ClearanceTest()
    if not GOAL_ACHEIVED:
        if Urgency_Report["act"]:
            Steer(vel)
            pub.publish(vel)
            print("Obstacle Detected")
        else:
            print("Obstacle not Detected")
            # vel.linear.x = NORMAL_LIN_VEL
            # vel.linear.y = 0
            # vel.linear.z = 0
            # vel.angular.x = 0
            # vel.angular.y = 0
            # vel.angular.z = 0
            pub.publish(goal_vel)

def Steer(velocity):
    global Urgency_Report
    #since we're moving only on the plane, all we need is move in the x axis,
    #and rotate in the z (zeta) axis.
    velocity.linear.x = TRANS_LIN_VEL
    velocity.linear.y = 0
    velocity.linear.z = 0
    velocity.angular.x = 0
    velocity.angular.y = 0
    velocity.angular.z = Urgency_Report["angular_vel"]
    # print(Urgency_Report["angular_vel"])

    # return velocity

GOAL = [3,1.5]  # Goal

wgain = 5.0 # Gain for the angular velocity [rad/s / rad]

distThresh = 0.1 # Distance treshold [m]

def findRegion(angle):
    region_distance=(int(round(angle/0.52)))
    prev_dist = region_distance -1 if region_distance > -5 else 6
    next_dist = region_distance +1 if region_distance < 6 else -5
    regions = ["","",""]
    for region,distance in Regions_Distances.items():
        if distance == region_distance:
            regions[1]= region
    for region,distance in Regions_Distances.items():
        if distance == prev_dist:
            regions[0]= region
    for region,distance in Regions_Distances.items():
        if distance == next_dist:
            regions[2]= region
    return regions

def recordOdom(message):
    global GOAL_ACHEIVED
    global GOAL_ORIENTATION

    # Generate a simplified pose
    pos = message.pose.pose
    quat = pos.orientation
    # From quaternion to Euler
    angles = tf.transformations.euler_from_quaternion((quat.x,quat.y,
                                                       quat.z,quat.w))
    theta = angles[2]
    pose = [pos.position.x, pos.position.y, theta]  # X, Y, Theta 
    
    # Proportional Controller
    v = 0 # default linear velocity
    w = 0 # default angluar velocity
    distance = sqrt((pose[0]-GOAL[0])**2+(pose[1]-GOAL[1])**2)
    if not GOAL_ACHEIVED:
        if (distance > distThresh):  
            v = NORMAL_LIN_VEL
            desireYaw = atan2(GOAL[1]-pose[1],GOAL[0]-pose[0])
            u = desireYaw-theta
            bound = atan2(sin(u),cos(u))
            w = min(0.5 , max(-0.5, wgain*bound))
            GOAL_ORIENTATION = min(2.6 , max(-2.6, wgain*bound))
            if len(Regions_Report[findRegion(w)[0]])>0 or len(Regions_Report[findRegion(w)[1]])>0  or len(Regions_Report[findRegion(w)[2]])>0:
                w = 0
            if abs(w)<0.17 :
                v *= 2
            if abs(w)>0.17 and distance < distThresh*2:
                v = TRANS_ANG_VEL
            if distance < distThresh*8 and distance > distThresh*6:
                v = 0.75
            if distance < distThresh*6 and distance > distThresh*4:
                v = 0.5
            if distance < distThresh*4 and distance > distThresh*2:
                v = 0.25

        else :
            GOAL_ACHEIVED = True
            print("Robot Arrived at Destination")
            print("--- %s seconds ---" % (time.time() - start_time))

        # Publish
        goal_vel.linear.x = v
        goal_vel.angular.z = w
    
    # Reporting
    # print('New Odom: x=%4.1f,y=%4.1f dist=%4.2f, cmd.v=%4.2f, cmd.w=%4.2f'%(pose[0],pose[1],distance,v,w))

def main():
    global vel
    global pub
    global cmd_pub
    global goal_vel
    global start_time
    start_time = time.time()

    #Initialize our node
    rospy.init_node("Laser_Obs_Avoid_node")
    #Subscribe to the "/scan" topic in order to read laser scans data from it
    rospy.Subscriber("/scan", LaserScan, IdentifyRegions)
    #create our publisher that'll publish to the "/cmd_vel" topic
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    vel = Twist()
    goal_vel = Twist()


    # Setup subscription - which implemets our controller.
    # We pass the publisher, the message to publish and the goal as 
    # additional parameters to the callback function.
    rospy.Subscriber('odometry/filtered',Odometry,recordOdom)
    #ros will try to run this code 10 times/second
    rospy.spin()
    # rate = rospy.Rate(10) #10Hz
    # #keep running while the ros-master isn't isn't shutdown
    # while not rospy.is_shutdown():

    #     # Need a do{ ... }while(); here (C is awesome)
    #     # Since I need to check at least once the clearance 
    #     # done = False
    #     # while not done:
    #     #     ClearanceTest()
    #     #     if(Urgency_Report["act"]):
    #     #         vel = Steer(vel)
    #     #         pub.publish(vel)
    #     #     else:
    #     #         done = True
    #     # # This else belongs to the while(), and the code below it could be cleaned furthermore
    #     # else: 
    #     #     vel.linear.x = NORMAL_LIN_VEL
    #     #     vel.linear.y = 0
    #     #     vel.linear.z = 0
    #     #     vel.angular.x = 0
    #     #     vel.angular.y = 0
    #     #     vel.angular.z = 0
    #     #     pub.publish(vel)

    #     ### This is stupid and shouldn't be done (sleep()) !
    #     # After publishing our action, we give it some time to execute the
    #     # needed actions before reading the data again.
    #     # time.sleep(Urgency_Report["sleep"])
        
    #     rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
