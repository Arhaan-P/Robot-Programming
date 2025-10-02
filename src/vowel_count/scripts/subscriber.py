#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(msg):
    text = msg.data
    count = sum(1 for c in text.lower() if c in 'aeiou')
    rospy.loginfo(f"Input: {text} | Vowels: {count}")

def main():
    rospy.init_node('typing_subscriber')
    rospy.Subscriber('typed_line', String, callback)
    rospy.spin()

if __name__ == "__main__":
    main()
