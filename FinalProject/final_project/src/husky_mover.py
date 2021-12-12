#!/usr/bin/env python
 
import sys
import rospy
from projectserver.srv import *
import math
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations  import euler_from_quaternion, quaternion_from_euler
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import cv2
import numpy as np
from std_msgs.msg import Float32
from collections import deque
import argparse
import imutils
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import OccupancyGrid
import heapq as hq
from geometry_msgs.msg import Twist, Pose
import rospy
import copy
import math




def detect_color():
    global cv_image
    if cv_image.data:
        red_mask_image = cv_image.copy()
        green_mask_image= cv_image.copy()
        hsv = cv2.cvtColor(red_mask_image, cv2.COLOR_BGR2HSV)

        masks=[]
        lower1 = np.array([0, 100, 20])
        upper1 = np.array([10, 255, 70])
        
        # upper boundary RED color range values; Hue (160 - 180)
        lower2 = np.array([160,100,20])
        upper2 = np.array([179,255,70])
        
        lower_mask = cv2.inRange(hsv, lower1, upper1)
        upper_mask = cv2.inRange(hsv, lower2, upper2)
        
        masks.append(lower_mask + upper_mask)

        green_lower = (40,40,40)
        green_upper = (75,255,255)
        hsv = cv2.cvtColor(green_mask_image, cv2.COLOR_BGR2HSV)
        masks.append(cv2.inRange(hsv, green_lower, green_upper))
        

        for i in range(len(masks)):
            mask = masks[i]
            # cv2.imshow('mask', mask)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)[-2]
            center = None
            if len(cnts) > 0:
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                print(center)
                if radius > 30:
                    print("color  detected")
                    return (i+1)
    return 0





# roll = pitch = yaw = 0.0
x = y = theta = 0.0

def get_rotation (msg):
    global x, y, theta 
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    euler = euler_from_quaternion (orientation_list)
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    theta = euler[2]
    print('X =',round(float(x),1), 'Y =',round(float(y),1), 'theta =',round(float(theta),1))


SIMILARITY_THRESHOLD = 0.1
SAFETY_OFFSET = 5    # number of pixels away from the wall the robot should remain


class Node:
    def __init__(self, x, y, theta=0.0, parent=None):
        self.x = x
        self.y = y
        self.theta = theta
        self.parent = parent
        # f(n) = h(n) + g(n)
        self.f = 0
        self.h = 0
        self.g = 0
        self.resolution = 0.05

    def euclidean_distance(self, goal):
        return math.sqrt(math.pow((goal.x-self.x),2) + math.pow((goal.y-self.y),2))

    def apply_move(self, move):
        theta_new = self.theta + move[1]
        x_new = self.x + math.cos(theta_new) * move[0]    # d.cos(theta)
        y_new = self.y + math.sin(theta_new) * move[0]  # d.sin(theta)
        return Node(x_new, y_new, theta_new)
    def is_allowed(self,state,grid_map):
        was_error = False
        i, j = state[0],state[1]
        side = int(math.floor((max(2, 3) / self.resolution) / 2))
        try:
            for s_i in range(i-side, i+side):
                for s_j in range(j-side, j+side):
                    cell = grid_map[s_i][s_j]
                    if cell == 100 or cell == -1:
                        return False
        except IndexError as e:
            # rospy.loginfo("Indices are out of range")
            was_error = True
        return True and not was_error
    def is_move_valid(self, grid_map, move):
        goal = self.apply_move(move)
        # convert goal coordinates to pixel coordinates before checking this
        goal_pixel = self.world_to_pixel((goal.x, goal.y), (-50, -50),self.resolution)
        # check if too close to the walls
        # if not is_allowed(goal_pixel,grid_map):
        #     return false
        # if (goal_pixel[0] <= SAFETY_OFFSET or goal_pixel[1] <= SAFETY_OFFSET)  and not is_allowed(goal_pixel,grid_map):#grid_map[goal_pixel[0]-SAFETY_OFFSET][goal_pixel[1]]:
        #     return False
        # # if goal_pixel[1] >= SAFETY_OFFSET and not grid_map[goal_pixel[0]][goal_pixel[1]-SAFETY_OFFSET]:
        # #     return False
        # if goal_pixel[0] >= SAFETY_OFFSET and goal_pixel[1] >= SAFETY_OFFSET and not grid_map[goal_pixel[0]-SAFETY_OFFSET][goal_pixel[1]-SAFETY_OFFSET]:
        #     return False
        # if grid_map[goal_pixel[0]][goal_pixel[1]]:
        #     return True
        return self.is_allowed(goal_pixel,grid_map)
    def world_to_pixel(self,pos,origin,resolution):
        pixel_points = [0,0]
        pixel_points[0] = int((pos[0] - origin[0]) / resolution)
        pixel_points[1] = int((pos[1] - origin[1]) / resolution)
        return pixel_points

    def is_valid(self, grid_map):
        """
        Return true if the location on the map is valid, ie, in obstacle free zone
        """
        goal_pixel = self.world_to_pixel((self.x, self.y),(-50, -50),0.05)
        if grid_map[goal_pixel[0]][goal_pixel[1]] != -1:
            return True
        return False

    def is_similar(self, other):
        """
        Return true if other node is in similar position as current node
        """
        return self.euclidean_distance(other) <= SIMILARITY_THRESHOLD
def radians(degree):
    return (degree * math.pi / 180)

G_MULTIPLIER = 0.2
MOVES = [ (0.2, radians(0)),     # move ahead
          (-0.2, radians(0)),     # move backwards
          (0, radians(90)),     # turn left
          (0, -radians(90)) ]    # turn right
TOLERANCE = 0.2

def move_robot(x,y):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

class PathPlanner:
    def __init__(self, start, theta, goal,grid):
        print("building map....")
        # map remains constant
        self.map = grid
        self.start = start
        self.theta = theta
        self.goal = goal
        print(np.shape(grid))
        print("map built. planner initialized")
    def a_star(self,start, end, grid_map):

        if not end.is_valid(grid_map):
            print("goal invalid")
            return None
        print("goal valid")
        opened = []
        closed=[]
        final = None
        hq.heappush(opened, (0.0, start))

        while (final == None) and opened:
            # q is a Node object with x, y, theta
            q = hq.heappop(opened)[1]
            for move in MOVES:        # move is in world coordinates
                if (q.is_move_valid(grid_map, move)):
                    next_node = q.apply_move(move)    # Node is returned in world coordinates
                else:
                    next_node = None
                #print("next node is : ", next_node) 
                if next_node != None:
                    if next_node.euclidean_distance(end) < TOLERANCE:
                        next_node.parent = q                    
                        final = next_node
                        break
                    # update heuristics h(n) and g(n)
                    next_node.h = next_node.euclidean_distance(end)
                    next_node.g = q.g + next_node.euclidean_distance(q)
                    # f(n) = h(n) + g(n)
                    next_node.f = G_MULTIPLIER * next_node.g + next_node.h
                    next_node.parent = q

                    # other candidate locations to put in the heap
                    potential_open = any(other_f <= next_node.f and other_next.is_similar(next_node) for other_f, other_next in opened)
                    
                    if not potential_open:
                        potential_closed = any(other_next.is_similar(next_node) and other_next.f <= next_node.f for other_next in closed)
                        if not potential_closed:
                            hq.heappush(opened, (next_node.f, next_node))
            closed.append(q)    

        return final   

    def plan(self):
        final = self.a_star(self.start, self.goal, self.map)
        if final == None:
            print("Path not found.")
        else:
            print("Constructing path..")
            path = self.construct_path(final)    # path in world coordinates
            print("path: ")
            points = []
            for step in path:
                points.append((step.x, step.y))
            # publish this path - safegoto for each of the path components
            points.reverse()
            points = points[1:]
            points.append((self.goal.x, self.goal.y))
            for p in range(len(points)):
                print("x:", points[p][0], " y:", points[p][1])
            # first process the points
            translate_x = points[0][0]
            translate_y = points[0][1]
            for p in range(len(points)):
                new_x = points[p][0] - translate_x
                new_y = points[p][1] - translate_y
                if self.theta == math.pi/2:
                    points[p] = [-new_y, new_x]
                elif self.theta == math.pi:
                    points[p] = [-new_x, -new_y]
                elif self.theta == -math.pi/2:
                    points[p] = [new_y, -new_x]
                else:            
                    points[p] = [new_x, new_y]
            # translate coordinates for theta            
            
                
            # run safegoto on the translated coordinates
            robot = SafeGoTo()
            robot.travel(points)
            return True


    def construct_path(self, end):
        """
        backtrack from end to construct path
        """
        current = end
        path = []    # path needs to be in world coordinates
        while current != None:
            path.append(current)
            current = current.parent
        return path


 
LINEAR_VELOCITY = 0.2
ANGULAR_VELOCITY = 0.4
TOLERANCE = 0.3
ROBOT_RADIUS = 0.22
OBSTACLE_THRESHOLD = 0.78
EXIT_STATUS_ERROR = 1
EXIT_STATUS_OK = 0

class SafeGoTo:
    def __init__(self):
        rospy.init_node('traveler', anonymous=True)
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Hold position and quaternion of robot            
        self.pos = Pose()
        self.theta = 0
        self.obstacle_found = False
        self.obstacle_circumventing = False
        self.start = (0, 0)
        self.goal = None
        self.mline = None
        self.curr_line = None
        self.sonar_data = []
        self.vel_msg = Twist()
        self.rate = rospy.Rate(10)
    

    def slope(self, p1, p2):
        delta_y = p2[1]-p1[1]
        delta_x = p2[0]-p1[0]
        return delta_y/delta_x if delta_x!=0 else float('inf')

    def euclidean_distance(self):
        return math.sqrt(math.pow((self.goal[0]-self.pos.position.x),2) + math.pow((self.goal[1]-self.pos.position.y),2))

    def angular_difference(self):
        return math.atan2(self.goal[1]-self.pos.position.y, self.goal[0]-self.pos.position.x) - self.theta

    
    def stop(self):

        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.vel_publisher.publish(self.vel_msg)    


    def go(self):
        
        # keep traveling until distance from current position to goal is greater than 0        
        while self.euclidean_distance() > TOLERANCE and not self.obstacle_found:
            #print("distance from goal " + str(self.goal) + ": ", self.euclidean_distance())
            # set linear velocity            
            self.vel_msg.linear.x = min(LINEAR_VELOCITY, 
                                        LINEAR_VELOCITY * self.euclidean_distance())
            self.vel_msg.linear.y = 0
            self.vel_msg.linear.z = 0
            # set angular velocity
            self.vel_msg.angular.x = 0
            self.vel_msg.angular.y = 0
            self.vel_msg.angular.z = min(ANGULAR_VELOCITY, 
                                         ANGULAR_VELOCITY * self.angular_difference())
            # publish velocity
            self.vel_publisher.publish(self.vel_msg)
            self.rate.sleep()
        
        if self.obstacle_found:
            self.stop()            
            # self.bug()    
        else:
            self.stop()
            self.start = self.goal

    def travel(self, goals):
        for goal in goals:
            self.start = (self.pos.position.x, self.pos.position.y)            
            self.mline = self.slope(self.start, goal)
            self.goal = goal            
            self.go()
            message = "Position " + str(self.goal) + " has been achieved."
            rospy.loginfo(message)

def Final_ints_client(firstwp,symboldetected,symbolcolor,symbolpositionx,symbolpositiony):    
    rospy.wait_for_service('Final_ints')
    try:
        Final_ints = rospy.ServiceProxy('Final_ints', getwaypoint)

        resp1 = Final_ints(firstwp,symboldetected,symbolcolor,symbolpositionx,symbolpositiony)
        return resp1
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)


map_grid = []
def grid_callback(data):
    global map_grid
    map_grid = np.array(data.data)
    # print(data.info)
    map_grid = map_grid.reshape(data.info.width,data.info.height,order='F')
    print("In subscription: ",np.shape(map_grid))

cv_image = Image()
def image_callback(data):
    global cv_image
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

def main():
    global vel
    global pub
    global bridge
    global map_grid
    #Initialize our node
    rospy.init_node("husky_mover")
    #create our publisher that'll publish to the "/cmd_vel" topic
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    vel = Twist()
    vel.linear.x = 0
    vel.linear.y = 0
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    #create our subscriber that'll subscribe to the "/amcl_pose" topic
    rospy.Subscriber ('/amcl_pose', PoseWithCovarianceStamped, get_rotation) # geometry_msgs/PoseWithCovariance pose
    try:
        bridge = CvBridge()
        image_sub = rospy.Subscriber("realsense/color/image_raw",Image,image_callback)
    except rospy.ROSInterruptException:
        video_capture.release()
        cv2.destroyAllWindows()
        pass
    rospy.Subscriber("/map", OccupancyGrid, grid_callback)
    firstwp = True
    symboldetected = False
    symbolcolor = 0
    symbolpositionx = 0
    symbolpositiony = 0
    r = rospy.Rate(10)
    while firstwp  == True or (result.waypointx != 0.0 and result.waypointy!=0.0):
        result = Final_ints_client(firstwp,symboldetected,symbolcolor,symbolpositionx,symbolpositiony)
        result.waypointx = round(result.waypointx,1)
        result.waypointy = round(result.waypointy,1)
        print("Next Position: ",(result.waypointx,result.waypointy))
        result_move = move_robot(result.waypointx,result.waypointy)
        start = Node(x, y, theta)
        goal = Node(result.waypointx , result.waypointx , math.pi)
        delt = 0
        while len(map_grid) == 0:
            delt = delt + 1
        # planner = PathPlanner(start, math.pi, goal,map_grid)
        # result_move = planner.plan()
        if result_move:
            print("Arrived at Goal!")
            color = 0
            start_theta = theta
            for w in np.arange(0,2*math.pi,0.1):
                # if (w > math.pi)
                vel.angular.z = w+start_theta
                pub.publish(vel)
                color  = detect_color()
                if color > 0 : 
                    break
                r.sleep()
            firstwp = False
            symboldetected = True
            # color  = detect_color()
            print("detected color : "+"RED" if color == 1 else ( "Green" if color ==2 else "No color Detected"))
            symbolcolor = color
            symbolpositionx = round(float(x),1)
            symbolpositiony = round(float(y),1)        
        # r.sleep()

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
