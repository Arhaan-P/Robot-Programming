#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math

def move_figure8():
    rospy.init_node('move_figure8', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    
    rospy.loginfo("Moving in figure-8...")
    
    twist = Twist()
    
    # First circle (counter-clockwise)
    rospy.loginfo("Drawing first circle...")
    for i in range(100):
        twist.linear.x = 2.0
        twist.angular.z = 1.0
        pub.publish(twist)
        rate.sleep()
    
    # Second circle (clockwise)
    rospy.loginfo("Drawing second circle...")
    for i in range(100):
        twist.linear.x = 2.0
        twist.angular.z = -1.0  # Negative for opposite direction
        pub.publish(twist)
        rate.sleep()
    
    # Stop
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)
    rospy.loginfo("Figure-8 complete!")

if __name__ == '__main__':
    try:
        move_figure8()
    except rospy.ROSInterruptException:
        pass
