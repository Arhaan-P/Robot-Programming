#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random

class WanderBot:
    def __init__(self):
        rospy.init_node('wander_bot', anonymous=True)
        
        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        # Parameters
        self.obstacle_distance = 0.8  # Stop if obstacle within 0.8m
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        
        # State
        self.obstacle_detected = False
        
        rospy.loginfo("WanderBot Started! Exploring...")
    
    def laser_callback(self, msg):
        # Check front region for obstacles
        # LaserScan typically has 360 readings (0-359 degrees)
        # Front is around index 0 (and 359)
        
        # Get front region (±30 degrees)
        front_ranges = []
        
        # Front center (0 degrees)
        front_ranges.extend(msg.ranges[0:30])
        front_ranges.extend(msg.ranges[-30:])
        
        # Filter out invalid readings (inf, nan)
        valid_ranges = [r for r in front_ranges if r > msg.range_min and r < msg.range_max]
        
        if len(valid_ranges) > 0:
            min_distance = min(valid_ranges)
            
            if min_distance < self.obstacle_distance:
                self.obstacle_detected = True
                rospy.logwarn(f"Obstacle detected at {min_distance:.2f}m - Turning!")
            else:
                self.obstacle_detected = False
        else:
            self.obstacle_detected = False
        
        self.move()
    
    def move(self):
        twist = Twist()
        
        if self.obstacle_detected:
            # Turn in random direction
            twist.linear.x = 0.0
            twist.angular.z = random.choice([-1, 1]) * self.angular_speed
        else:
            # Move forward
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        bot = WanderBot()
        bot.run()
    except rospy.ROSInterruptException:
        pass
