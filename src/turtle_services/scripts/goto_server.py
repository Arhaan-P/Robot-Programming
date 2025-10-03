#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtle_services.srv import GoToPoint, GoToPointResponse
import math

current_pose = Pose()

def pose_callback(msg):
    global current_pose
    current_pose = msg

def handle_go_to_point(req):
    rospy.loginfo(f"Received request to go to ({req.x}, {req.y})")
    
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    
    twist = Twist()
    
    while not rospy.is_shutdown():
        # Calculate distance
        dx = req.x - current_pose.x
        dy = req.y - current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Check if reached
        if distance < 0.1:
            break
        
        # Calculate angle
        desired_angle = math.atan2(dy, dx)
        angle_diff = desired_angle - current_pose.theta
        
        # Normalize angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Control
        twist.linear.x = min(1.5 * distance, 2.0)
        twist.angular.z = 4.0 * angle_diff
        
        pub.publish(twist)
        rate.sleep()
    
    # Stop
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)
    
    rospy.loginfo(f"Reached goal ({req.x}, {req.y})")
    return GoToPointResponse(True, f"Successfully reached ({req.x}, {req.y})")

def go_to_point_server():
    rospy.init_node('go_to_point_server')
    
    # Subscribe to turtle pose
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    
    # Create service
    service = rospy.Service('go_to_point', GoToPoint, handle_go_to_point)
    
    rospy.loginfo("Go To Point Service is ready!")
    rospy.spin()

if __name__ == '__main__':
    go_to_point_server()
