#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32

def location_cb(msg):
    rospy.loginfo(f"[Sub1] Location: {msg.data}")

def status_cb(msg):
    rospy.loginfo(f"[Sub1] Status: {msg.data}")

def weight_cb(msg):
    rospy.loginfo(f"[Sub1] Weight: {msg.data} kg")

def main():
    rospy.init_node('subscriber1')
    rospy.Subscriber('/location', String, location_cb)
    rospy.Subscriber('/status', String, status_cb)
    rospy.Subscriber('/weight', Float32, weight_cb)
    rospy.spin()

if __name__ == '__main__':
    main()

