#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String

class CrowdCheer:
    def __init__(self):
        rospy.init_node('crowd_cheer', anonymous=True)
        self.action_pub = rospy.Publisher('/crowd_action', String, queue_size=10)
        rospy.Subscriber('/crowd_noise', Float32, self.noise_callback)
        self.actions = {
            'quiet': "Chill mode!",
            'lively': "Party time!",
            'crazy': "Go crazy!"
        }
        rospy.loginfo("CrowdCheer Started. Let's hype the audience!")

    def noise_callback(self, data):
        noise = data.data
        if noise < 0.3:
            action = self.actions['quiet']
        elif noise < 0.7:
            action = self.actions['lively']
        else:
            action = self.actions['crazy']
        self.action_pub.publish(action)
        rospy.loginfo(f"Noise Level: {noise:.2f} - {action}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        CrowdCheer().run()
    except rospy.ROSInterruptException:
        pass
