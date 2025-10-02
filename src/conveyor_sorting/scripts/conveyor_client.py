#!/usr/bin/env python3

import rospy
import actionlib
from conveyor_sorting.msg import ConveyorSortingAction, ConveyorSortingGoal

def feedback_cb(feedback):
    rospy.loginfo(f"📊 Progress: {feedback.percent_complete:.1f}% - {feedback.current_stage}")

def conveyor_client():
    rospy.init_node('conveyor_sorting_client')
    
    # Create action client
    client = actionlib.SimpleActionClient('conveyor_sorting', ConveyorSortingAction)
    
    rospy.loginfo("🔌 Connecting to conveyor sorting server...")
    client.wait_for_server()
    rospy.loginfo("✅ Connected to server!")
    
    # Test different objects
    test_objects = [
        {"object_id": "plastic_bottle", "target_bin": 1},
        {"object_id": "metal_can", "target_bin": 2}, 
        {"object_id": "glass_jar", "target_bin": 3},
        {"object_id": "electronic_waste", "target_bin": 4},
        {"object_id": "mystery_item", "target_bin": 5}
    ]
    
    for i, obj in enumerate(test_objects):
        rospy.loginfo(f"\n🚀 Test {i+1}/{len(test_objects)}: Sorting {obj['object_id']}")
        
        # Create goal
        goal = ConveyorSortingGoal()
        goal.object_id = obj["object_id"]
        goal.target_bin = obj["target_bin"]
        
        # Send goal
        client.send_goal(goal, feedback_cb=feedback_cb)
        
        # Wait for result
        client.wait_for_result()
        result = client.get_result()
        
        rospy.loginfo(f"📋 Final Result: {result.status}")
        rospy.loginfo("="*50)
        
        if i < len(test_objects) - 1:
            rospy.sleep(2)  # Brief pause between tests

if __name__ == '__main__':
    try:
        conveyor_client()
    except rospy.ROSInterruptException:
        pass