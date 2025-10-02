#!/usr/bin/env python3

import rospy
import math
import threading
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class Figure8Service:
    def __init__(self):
        rospy.init_node('figure8_service_server')
        
        # Publisher for turtle velocity
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Subscriber for turtle pose
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Service server
        self.service = rospy.Service('start_figure8', Empty, self.start_figure8)
        
        # Control variables
        self.current_pose = None
        self.figure8_active = False
        self.figure8_thread = None
        
        rospy.loginfo("Figure-8 Service Server Ready!")
        rospy.loginfo("Call 'rosservice call /start_figure8' to start the infinity pattern")
        
    def pose_callback(self, msg):
        self.current_pose = msg
        
    def start_figure8(self, req):
        if not self.figure8_active:
            rospy.loginfo("Starting Figure-8 infinity pattern!")
            self.figure8_active = True
            
            # Start figure-8 in a separate thread
            if self.figure8_thread is None or not self.figure8_thread.is_alive():
                self.figure8_thread = threading.Thread(target=self.execute_figure8)
                self.figure8_thread.daemon = True
                self.figure8_thread.start()
            
            return EmptyResponse()
        else:
            rospy.loginfo("Figure-8 pattern already running!")
            return EmptyResponse()
    
    def execute_figure8(self):
        """Execute infinite figure-8 pattern"""
        rate = rospy.Rate(50)  # 50 Hz for smooth motion
        t = 0.0
        
        rospy.loginfo("🚀 Starting infinite Figure-8 pattern! Press Ctrl+C to stop.")
        
        while not rospy.is_shutdown() and self.figure8_active:
            # Figure-8 parametric equations
            # x = sin(t), y = sin(2*t)/2
            # Scale and adjust for turtlesim window (11x11 units)
            
            scale = 2.0
            center_x = 5.5
            center_y = 5.5
            
            # Calculate desired position
            target_x = center_x + scale * math.sin(t)
            target_y = center_y + scale * math.sin(2 * t) / 2
            
            # Calculate derivatives for velocity (tangent to curve)
            dx_dt = scale * math.cos(t)
            dy_dt = scale * math.cos(2 * t)
            
            # Create and publish velocity command
            twist = Twist()
            speed_factor = 1.5
            twist.linear.x = speed_factor * math.sqrt(dx_dt**2 + dy_dt**2)
            twist.angular.z = speed_factor * (dy_dt * dx_dt - dx_dt * dy_dt) / (dx_dt**2 + dy_dt**2 + 1e-6)
            
            # Adjust angular velocity for proper figure-8
            twist.angular.z = speed_factor * 2 * math.cos(2 * t)
            
            self.cmd_vel_pub.publish(twist)
            
            # Log progress occasionally
            if int(t * 10) % 50 == 0:  # Every 5 seconds approximately
                rospy.loginfo(f"📍 Figure-8 progress: t={t:.2f}, completing infinity loop...")
            
            t += 0.02  # Increment time
            rate.sleep()
        
        # Stop the turtle when done
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        rospy.loginfo("Figure-8 pattern stopped.")

if __name__ == '__main__':
    try:
        service = Figure8Service()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass