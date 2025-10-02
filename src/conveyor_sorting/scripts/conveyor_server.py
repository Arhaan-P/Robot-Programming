#!/usr/bin/env python3

import rospy
import actionlib
import time
import random
from conveyor_sorting.msg import ConveyorSortingAction, ConveyorSortingGoal, ConveyorSortingResult, ConveyorSortingFeedback

class ConveyorSortingServer:
    def __init__(self):
        rospy.init_node('conveyor_sorting_server')
        
        # Create action server
        self.server = actionlib.SimpleActionServer(
            'conveyor_sorting', 
            ConveyorSortingAction, 
            self.execute_sorting, 
            False
        )
        
        self.server.start()
        rospy.loginfo("🤖 Conveyor Belt Sorting Action Server Started!")
        rospy.loginfo("📦 Ready to sort objects into bins!")
        
        # Simulate different object types and sorting complexity
        self.object_types = {
            "plastic_bottle": {"time": 3.0, "complexity": "Easy"},
            "metal_can": {"time": 4.0, "complexity": "Medium"}, 
            "glass_jar": {"time": 6.0, "complexity": "Hard"},
            "cardboard_box": {"time": 2.5, "complexity": "Easy"},
            "electronic_waste": {"time": 8.0, "complexity": "Very Hard"},
            "unknown_object": {"time": 5.0, "complexity": "Medium"}
        }
        
        self.sorting_stages = [
            "Detecting object on conveyor",
            "Analyzing object properties", 
            "Calculating optimal path",
            "Activating sorting arm",
            "Moving to target bin",
            "Releasing object",
            "Returning to home position",
            "Verifying sort completion"
        ]
        
    def execute_sorting(self, goal):
        rospy.loginfo(f"🎯 NEW SORTING REQUEST:")
        rospy.loginfo(f"   Object ID: {goal.object_id}")
        rospy.loginfo(f"   Target Bin: {goal.target_bin}")
        
        # Validate target bin
        if goal.target_bin < 1 or goal.target_bin > 5:
            rospy.logwarn(f"❌ Invalid target bin: {goal.target_bin}. Valid range: 1-5")
            result = ConveyorSortingResult()
            result.status = f"FAILED: Invalid target bin {goal.target_bin}"
            self.server.set_aborted(result)
            return
            
        # Get object info or use unknown
        object_info = self.object_types.get(goal.object_id.lower(), self.object_types["unknown_object"])
        total_time = object_info["time"]
        complexity = object_info["complexity"]
        
        rospy.loginfo(f"📊 Object Analysis:")
        rospy.loginfo(f"   Type: {goal.object_id}")
        rospy.loginfo(f"   Complexity: {complexity}")
        rospy.loginfo(f"   Estimated Time: {total_time:.1f}s")
        
        # Execute sorting process with detailed feedback
        feedback = ConveyorSortingFeedback()
        rate = rospy.Rate(10)  # 10 Hz updates
        
        start_time = time.time()
        
        for i, stage in enumerate(self.sorting_stages):
            if self.server.is_preempt_requested():
                rospy.loginfo("🛑 Sorting operation preempted!")
                self.server.set_preempted()
                return
                
            stage_progress = (i / len(self.sorting_stages)) * 100
            feedback.percent_complete = stage_progress
            feedback.current_stage = stage
            self.server.publish_feedback(feedback)
            
            rospy.loginfo(f"🔄 [{stage_progress:5.1f}%] {stage}")
            
            # Simulate different stage durations
            stage_duration = total_time / len(self.sorting_stages)
            
            # Add some randomization for realism
            if "arm" in stage.lower():
                stage_duration *= 1.5  # Mechanical movements take longer
            elif "analyzing" in stage.lower():
                stage_duration *= 1.2  # Analysis takes extra time
                
            # Simulate potential issues
            if random.random() < 0.1:  # 10% chance of minor delay
                rospy.logwarn(f"⚠️  Minor delay during: {stage}")
                stage_duration *= 1.3
                
            time.sleep(stage_duration)
            
        # Final completion
        feedback.percent_complete = 100.0
        feedback.current_stage = "Sort completed successfully"
        self.server.publish_feedback(feedback)
        
        elapsed_time = time.time() - start_time
        
        # Determine success/failure based on complexity
        success_rate = {"Easy": 0.98, "Medium": 0.92, "Hard": 0.85, "Very Hard": 0.75}
        is_success = random.random() < success_rate.get(complexity, 0.90)
        
        result = ConveyorSortingResult()
        
        if is_success:
            result.status = f"SUCCESS: {goal.object_id} sorted to bin {goal.target_bin}"
            rospy.loginfo(f"✅ SORTING COMPLETED!")
            rospy.loginfo(f"   Object: {goal.object_id} → Bin {goal.target_bin}")
            rospy.loginfo(f"   Duration: {elapsed_time:.2f}s")
            rospy.loginfo(f"   Status: {result.status}")
            self.server.set_succeeded(result)
        else:
            result.status = f"FAILED: Could not sort {goal.object_id} - {self.get_failure_reason()}"
            rospy.logwarn(f"❌ SORTING FAILED!")
            rospy.logwarn(f"   Object: {goal.object_id}")
            rospy.logwarn(f"   Reason: {result.status}")
            self.server.set_aborted(result)
            
    def get_failure_reason(self):
        reasons = [
            "Object slipped from gripper",
            "Bin full - overflow detected", 
            "Sensor calibration error",
            "Conveyor belt jam",
            "Object too fragile to handle"
        ]
        return random.choice(reasons)

if __name__ == '__main__':
    try:
        server = ConveyorSortingServer()
        rospy.loginfo("🏭 Conveyor sorting system online. Waiting for sorting requests...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Conveyor sorting server shutting down.")