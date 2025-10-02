#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('news_transit')
    pub = rospy.Publisher("/news_radio", String, queue_size=10)
    rate = rospy.Rate(2)  # 2 Hz
    while not rospy.is_shutdown():
        msg = String()
        msg.data = "Hi, This is Robo communicating"
        pub.publish(msg)
        rate.sleep()

