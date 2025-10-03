#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollower:
    def __init__(self):
        rospy.init_node('line_follower', anonymous=True)
        
        self.bridge = CvBridge()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        
        # Line detection parameters
        self.lower_threshold = np.array([0, 0, 0])      # Black line lower bound (HSV)
        self.upper_threshold = np.array([180, 255, 50])  # Black line upper bound (HSV)
        
        # Control parameters
        self.linear_speed = 0.2
        self.angular_gain = 0.01  # Proportional gain
        
        rospy.loginfo("Line Follower Started!")
    
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Get image dimensions
            height, width, _ = cv_image.shape
            
            # Focus on bottom half of image
            roi = cv_image[int(height/2):height, 0:width]
            
            # Convert to HSV color space
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            
            # Create mask for black line
            mask = cv2.inRange(hsv, self.lower_threshold, self.upper_threshold)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            twist = Twist()
            
            if len(contours) > 0:
                # Find largest contour (the line)
                largest_contour = max(contours, key=cv2.contourArea)
                
                # Calculate centroid
                M = cv2.moments(largest_contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    
                    # Calculate error (difference from center)
                    error = cx - width / 2
                    
                    # Proportional control
                    twist.linear.x = self.linear_speed
                    twist.angular.z = -self.angular_gain * error
                    
                    rospy.loginfo(f"Line detected at x={cx}, error={error:.1f}, angular.z={twist.angular.z:.3f}")
                else:
                    # No valid contour
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    rospy.logwarn("Line lost - stopping")
            else:
                # No line detected
                twist.linear.x = 0.0
                twist.angular.z = 0.3  # Slowly rotate to find line
                rospy.logwarn("Searching for line...")
            
            self.cmd_vel_pub.publish(twist)
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        follower = LineFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
