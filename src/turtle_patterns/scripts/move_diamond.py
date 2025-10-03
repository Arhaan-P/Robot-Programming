#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math

def move_diamond():
    rospy.init_node('move_diamond', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    rospy.loginfo("Moving in diamond...")
    
    twist = Twist()
    
    # Initial 45-degree rotation
    rospy.loginfo("Rotating to 45 degrees...")
    twist.angular.z = math.pi / 4
    pub.publish(twist)
    rospy.sleep(1)
    
    twist.angular.z = 0.0
    pub.publish(twist)
    rospy.sleep(0.5)
    
    # Draw square (appears as diamond due to initial rotation)
    for i in range(4):
        rospy.loginfo(f"Side {i+1}/4")
        
        # Forward
        twist.linear.x = 2.0
        pub.publish(twist)
        rospy.sleep(2)
        
        # Stop
        twist.linear.x = 0.0
        pub.publish(twist)
        rospy.sleep(0.5)
        
        # Turn 90 degrees
        twist.angular.z = math.pi / 2
        pub.publish(twist)
        rospy.sleep(1)
        
        # Stop
        twist.angular.z = 0.0
        pub.publish(twist)
        rospy.sleep(0.5)
    
    rospy.loginfo("Diamond complete!")

if __name__ == '__main__':
    try:
        move_diamond()
    except rospy.ROSInterruptException:
        pass
