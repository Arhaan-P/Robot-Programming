#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class ObstacleAvoider:
    def __init__(self):
        rospy.init_node('obstacle_avoider', anonymous=True)
        
        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.ultrasonic_sub = rospy.Subscriber('/ultrasonic', Range, self.ultrasonic_callback)
        
        # Parameters
        self.safe_distance = 0.5  # Stop if obstacle within 0.5m
        self.linear_speed = 0.3
        self.angular_speed = 0.8
        
        # State
        self.current_distance = float('inf')
        
        rospy.loginfo("Obstacle Avoider Started!")
    
    def ultrasonic_callback(self, msg):
        self.current_distance = msg.range
        
        # Validate reading
        if msg.range < msg.min_range or msg.range > msg.max_range:
            rospy.logwarn("Invalid ultrasonic reading!")
            return
        
        rospy.loginfo(f"Distance: {msg.range:.2f}m")
        self.avoid_obstacle()
    
    def avoid_obstacle(self):
        twist = Twist()
        
        if self.current_distance < self.safe_distance:
            # Obstacle detected - turn right
            twist.linear.x = 0.0
            twist.angular.z = -self.angular_speed
            rospy.logwarn(f"OBSTACLE at {self.current_distance:.2f}m - Turning!")
        else:
            # Path clear - move forward
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        avoider = ObstacleAvoider()
        avoider.run()
    except rospy.ROSInterruptException:
        pass
