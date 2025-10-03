#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_circle():
    rospy.init_node('move_circle', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    
    rospy.loginfo("Moving in circle...")
    
    twist = Twist()
    twist.linear.x = 2.0
    twist.angular.z = 1.0  # Non-zero angular creates circle
    
    while not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_circle()
    except rospy.ROSInterruptException:
        pass
