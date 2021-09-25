#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from husky_ehsan.msg import wheel_velocities

wheel_radius = 0.05
wheel_separation = 1

def callback(cmd_vel):
    rospy.loginfo(rospy.get_caller_id() + "Husky cmd_vel %s", cmd_vel.linear)
    wheel_velocity = wheel_velocities()
    wheel_velocity.left_wheel_velocity = (cmd_vel.linear.x/wheel_radius) - (wheel_separation/wheel_radius)*cmd_vel.angular.z
    wheel_velocity.right_wheel_velocity = (cmd_vel.linear.x/wheel_radius) + (wheel_separation/wheel_radius)*cmd_vel.angular.z
    rospy.loginfo("Wheel Velocities: %s",wheel_velocity)
    pub = rospy.Publisher('DIFF_IK_OUTPUT', wheel_velocities)
    pub.publish(wheel_velocity)
    

def listen_husky():
	rospy.init_node('listen_husky', anonymous=True)
	rospy.Subscriber("cmd_vel", Twist, callback)
	rospy.spin()

if __name__ == '__main__':
    try:
        listen_husky()
    except rospy.ROSInterruptException:
        pass
