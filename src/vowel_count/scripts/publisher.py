#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node('typing_publisher')
    pub = rospy.Publisher('typed_line', String, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    print("Start typing lines (press Ctrl+C to stop):")
    while not rospy.is_shutdown():
        line = input()
        pub.publish(line)
        rate.sleep()

if __name__ == "__main__":
    main()
