#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class LineFollower:
    def __init__(self):
        rospy.init_node('line_follower', anonymous=True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Publishers and subscribers
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        # Movement parameters
        self.twist = Twist()
        self.linear_speed = 0.3
        self.angular_speed = 0.8
        
        # Line following parameters
        self.cx = 0
        self.cy = 0
        self.junction_detected = False
        self.junction_threshold = 3000  # Pixel count threshold for junction detection
        
        rospy.loginfo("Line Follower Node Started!")

    def image_callback(self, data):
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CV Bridge Error: %s", e)
            return

        # Process the image for line detection
        self.process_image(cv_image)

    def process_image(self, image):
        # Get image dimensions
        height, width, channels = image.shape
        
        # Define region of interest (lower part of image)
        roi_height = int(height * 0.3)  # Bottom 30% of image
        roi = image[height - roi_height:height, 0:width]
        
        # Convert to grayscale
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # Apply binary threshold to detect black line
        _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        
        # Find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find the largest contour (should be the line)
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Check for junction (large contour area)
            contour_area = cv2.contourArea(largest_contour)
            
            if contour_area > self.junction_threshold:
                rospy.loginfo("Junction detected! Turning left...")
                self.junction_detected = True
                self.turn_left()
            else:
                self.junction_detected = False
                # Calculate centroid of the line
                M = cv2.moments(largest_contour)
                
                if M["m00"] != 0:
                    self.cx = int(M["m10"] / M["m00"])
                    self.cy = int(M["m01"] / M["m00"])
                    
                    # Follow the line
                    self.follow_line(width)
                else:
                    # Stop if no line detected
                    self.stop_robot()
        else:
            # No line detected, stop
            self.stop_robot()

    def follow_line(self, image_width):
        # Calculate error (deviation from center)
        center_x = image_width // 2
        error = self.cx - center_x
        
        # PID-like control for smooth following
        angular_z = -float(error) / 100.0
        
        # Limit angular velocity
        if angular_z > self.angular_speed:
            angular_z = self.angular_speed
        elif angular_z < -self.angular_speed:
            angular_z = -self.angular_speed
        
        # Set velocities
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = angular_z
        
        # Publish movement command
        self.cmd_vel_pub.publish(self.twist)
        
        rospy.loginfo("Following line - Error: %d, Angular: %.2f", error, angular_z)

    def turn_left(self):
        # Execute left turn at junction
        self.twist.linear.x = 0.1  # Slow forward movement
        self.twist.angular.z = self.angular_speed  # Turn left
        
        # Publish turn command
        self.cmd_vel_pub.publish(self.twist)
        
        # Turn for a specific duration
        rospy.sleep(2.0)  # Adjust timing as needed
        
        # Resume normal following
        self.twist.angular.z = 0

    def stop_robot(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
        rospy.loginfo("Robot stopped - No line detected")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        line_follower = LineFollower()
        line_follower.run()
    except rospy.ROSInterruptException:
        pass
