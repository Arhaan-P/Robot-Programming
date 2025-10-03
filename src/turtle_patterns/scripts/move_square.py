#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math

def move_square():
    rospy.init_node('move_square', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    rospy.loginfo("Moving in square...")
    
    twist = Twist()
    
    for i in range(5):
        
        # Move forward
        twist.linear.x = 2.0
        twist.angular.z = 0.0
        pub.publish(twist)
        rospy.sleep(2.0)
        
        # Stop
        twist.linear.x = 0.0
        pub.publish(twist)
        rospy.sleep(0.5)
        
        # Turn 90 degrees
        rospy.loginfo("Turning...")
        twist.angular.z = math.pi / 2
        pub.publish(twist)
        rospy.sleep(1.0)
        
        # Stop rotation
        twist.angular.z = 0.0
        pub.publish(twist)
        rospy.sleep(0.5)
    
    rospy.loginfo("Square complete!")

if __name__ == '__main__':
    try:
        move_square()
    except rospy.ROSInterruptException:
        pass
