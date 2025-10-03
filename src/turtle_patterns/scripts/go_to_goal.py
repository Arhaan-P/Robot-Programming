#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

current_pose = Pose()

def pose_callback(msg):
    global current_pose
    current_pose = msg

def go_to_goal(target_x, target_y):
    rospy.init_node('go_to_goal', anonymous=True)
    
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    
    rate = rospy.Rate(10)
    
    rospy.loginfo(f"Going to point ({target_x}, {target_y})...")
    
    twist = Twist()
    
    while not rospy.is_shutdown():
        # Calculate distance to goal
        dx = target_x - current_pose.x
        dy = target_y - current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Check if reached goal
        if distance < 0.1:
            rospy.loginfo("Goal reached!")
            break
        
        # Calculate desired angle
        desired_angle = math.atan2(dy, dx)
        angle_diff = desired_angle - current_pose.theta
        
        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Proportional control
        twist.linear.x = min(1.5 * distance, 2.0)  # Cap max speed
        twist.angular.z = 4.0 * angle_diff
        
        pub.publish(twist)
        rate.sleep()
    
    # Stop at goal
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)

if __name__ == '__main__':
    try:
        import sys
        if len(sys.argv) == 3:
            x = float(sys.argv[1])
            y = float(sys.argv[2])
        else:
            x, y = 8.0, 8.0  # Default goal
        
        go_to_goal(x, y)
    except rospy.ROSInterruptException:
        pass
