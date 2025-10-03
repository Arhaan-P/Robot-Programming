#!/usr/bin/env python3

import rospy
import actionlib
from turtle_actions.msg import NavigateToGoalAction, NavigateToGoalGoal

def feedback_callback(feedback):
    rospy.loginfo(f"Distance remaining: {feedback.current_distance:.2f}m, Progress: {feedback.percent_complete:.1f}%")

def navigate_to_goal(x, y):
    # Create action client
    client = actionlib.SimpleActionClient('navigate_to_goal', NavigateToGoalAction)
    
    rospy.loginfo("Waiting for action server...")
    client.wait_for_server()
    rospy.loginfo("Action server found!")
    
    # Create goal
    goal = NavigateToGoalGoal()
    goal.target_x = x
    goal.target_y = y
    
    rospy.loginfo(f"Sending goal: ({x}, {y})")
    client.send_goal(goal, feedback_cb=feedback_callback)
    
    # Wait for result
    client.wait_for_result()
    
    result = client.get_result()
    if result.success:
        rospy.loginfo(f"SUCCESS! Final distance: {result.final_distance:.4f}m")
    else:
        rospy.logwarn("Goal failed!")
    
    return result

if __name__ == '__main__':
    try:
        rospy.init_node('navigate_client')
        
        # Test multiple goals
        rospy.loginfo("=== TEST 1: Go to (8, 8) ===")
        navigate_to_goal(8.0, 8.0)
        rospy.sleep(1)
        
        rospy.loginfo("\n=== TEST 2: Go to (2, 2) ===")
        navigate_to_goal(2.0, 2.0)
        rospy.sleep(1)
        
        rospy.loginfo("\n=== TEST 3: Go to (5, 5) ===")
        navigate_to_goal(5.0, 5.0)
        
        rospy.loginfo("\nAll goals completed!")
        
    except rospy.ROSInterruptException:
        pass
