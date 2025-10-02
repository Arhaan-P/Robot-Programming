#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('news_broadcast')
    pub = rospy.Publisher("/news_radio", String, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        msg = String()
        msg.data = "Cross-language broadcast: Hello from Python!"
        pub.publish(msg)
        rospy.loginfo("Published: %s", msg.data)
        rate.sleep()

