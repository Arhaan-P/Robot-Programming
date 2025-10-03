#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range

def publish_ultrasonic():
    rospy.init_node('ultrasonic_simulator', anonymous=True)
    pub = rospy.Publisher('/ultrasonic', Range, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    
    rospy.loginfo("Ultrasonic Simulator Started!")
    rospy.loginfo("Publishing simulated ultrasonic data...")
    
    distance = 2.0  # Start at 2m
    direction = -1  # Moving closer
    
    while not rospy.is_shutdown():
        # Create Range message
        msg = Range()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "ultrasonic_sensor"
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.1  # ~6 degrees
        msg.min_range = 0.02  # 2cm
        msg.max_range = 4.0   # 4m
        msg.range = distance
        
        pub.publish(msg)
        rospy.loginfo(f"Distance: {distance:.2f}m")
        
        # Simulate object moving back and forth
        distance += direction * 0.05
        if distance < 0.2:
            direction = 1  # Start moving away
        elif distance > 2.0:
            direction = -1  # Start moving closer
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_ultrasonic()
    except rospy.ROSInterruptException:
        pass
