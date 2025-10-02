#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32

def location_cb(msg):
    rospy.loginfo(f"[Sub2] Location: {msg.data}")

def status_cb(msg):
    if "delayed" in msg.data.lower() or "hold" in msg.data.lower():
        rospy.logwarn(f"[Sub2 ALERT] Status problem: {msg.data}")
    else:
        rospy.loginfo(f"[Sub2] Status: {msg.data}")

def weight_cb(msg):
    if msg.data >= 8.0:
        rospy.logwarn(f"[Sub2 ALERT] Heavy weight = {msg.data} kg")
    else:
        rospy.loginfo(f"[Sub2] Weight: {msg.data} kg")

def main():
    rospy.init_node('subscriber2')
    rospy.Subscriber('/location', String, location_cb)
    rospy.Subscriber('/status', String, status_cb)
    rospy.Subscriber('/weight', Float32, weight_cb)
    rospy.spin()

if __name__ == '__main__':
    main()

