#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


def move_husky():
	pub = rospy.Publisher('cmd_vel', Twist)
	rospy.init_node('move_husky', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	move = Twist()
	while not rospy.is_shutdown():
	    rospy.loginfo("Motion Input: move husky with constant speed of 0.2m/s")
	    move.linear.x = 0.2
	    move.linear.y = 0
	    move.linear.z = 0
	    move.angular.z = 0
	    rospy.loginfo("checking for cmd\n" + str(move.linear))
	    pub.publish(move)
	    rate.sleep()

if __name__ == '__main__':
    try:
        move_husky()
    except rospy.ROSInterruptException:
        pass
