#!/usr/bin/env python3

import rospy
import actionlib
from turtle_actions.msg import NavigateToGoalAction, NavigateToGoalFeedback, NavigateToGoalResult
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

current_pose = Pose()

def pose_callback(msg):
    global current_pose
    current_pose = msg

class NavigateToGoalServer:
    def __init__(self):
        rospy.init_node('navigate_to_goal_server')
        
        # Subscribe to turtle pose
        rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
        
        # Create publisher
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Create action server
        self.server = actionlib.SimpleActionServer(
            'navigate_to_goal',
            NavigateToGoalAction,
            self.execute,
            False
        )
        self.server.start()
        rospy.loginfo("Navigate To Goal Action Server started!")
    
    def execute(self, goal):
        rospy.loginfo(f"Navigating to ({goal.target_x}, {goal.target_y})...")
        
        rate = rospy.Rate(10)
        feedback = NavigateToGoalFeedback()
        result = NavigateToGoalResult()
        twist = Twist()
        
        # Calculate initial distance
        initial_distance = math.sqrt(
            (goal.target_x - current_pose.x)**2 + 
            (goal.target_y - current_pose.y)**2
        )
        
        while not rospy.is_shutdown():
            # Check for preemption
            if self.server.is_preempt_requested():
                rospy.loginfo("Goal preempted")
                self.server.set_preempted()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                return
            
            # Calculate distance
            dx = goal.target_x - current_pose.x
            dy = goal.target_y - current_pose.y
            distance = math.sqrt(dx**2 + dy**2)
            
            # Check if reached goal
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
            
            self.cmd_vel_pub.publish(twist)
            
            # Send feedback
            feedback.current_distance = distance
            feedback.percent_complete = (1.0 - distance / initial_distance) * 100.0
            self.server.publish_feedback(feedback)
            
            rospy.loginfo(f"Distance: {distance:.2f}m, Progress: {feedback.percent_complete:.1f}%")
            
            rate.sleep()
        
        # Stop robot
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        # Send result
        result.success = True
        result.final_distance = distance
        self.server.set_succeeded(result)
        rospy.loginfo("Goal reached!")

if __name__ == '__main__':
    try:
        server = NavigateToGoalServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
