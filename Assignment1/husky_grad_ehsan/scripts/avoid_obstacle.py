#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def callback(msg):
	if msg.ranges[0] > 1:
	    move.linear.x = 0.2
	    move.angular.z = 0.0

	if msg.ranges[0] < 1:
	    move.linear.x = 0.2
	    move.angular.z = 0.2
	    rospy.loginfo("Obstacle Detected and Applied Rotation of of 0.2 rad/s")

	pub.publish(move)
  
def avoid_obstacle():
	global move
	global pub
	rospy.init_node('avoid_obstacle', anonymous=True)
	pub = rospy.Publisher('cmd_vel', Twist)
	sub = rospy.Subscriber('scan', LaserScan, callback)
	move = Twist()
	rospy.spin()

if __name__ == '__main__':
    try:
        avoid_obstacle()
    except rospy.ROSInterruptException:
        pass
