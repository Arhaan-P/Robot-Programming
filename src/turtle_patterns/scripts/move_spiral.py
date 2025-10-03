#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_spiral():
    rospy.init_node('move_spiral', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    
    rospy.loginfo("Moving in spiral...")
    
    twist = Twist()
    radius = 0.5
    angular_speed = 2.0
    
    while not rospy.is_shutdown() and radius < 10:
        # Spiral: linear velocity increases with radius
        twist.linear.x = radius * angular_speed
        twist.angular.z = angular_speed
        
        pub.publish(twist)
        radius += 0.01  # Gradually increase radius
        rate.sleep()
    
    # Stop at the end
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)
    rospy.loginfo("Spiral complete!")

if __name__ == '__main__':
    try:
        move_spiral()
    except rospy.ROSInterruptException:
        pass
