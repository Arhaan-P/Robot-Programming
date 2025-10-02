#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32

def main():
    rospy.init_node('publisher2')
    pub_location = rospy.Publisher('/location', String, queue_size=10)
    pub_status = rospy.Publisher('/status', String, queue_size=10)
    pub_weight = rospy.Publisher('/weight', Float32, queue_size=10)
    rate = rospy.Rate(1)

    locations = ['LoadingDock', 'Aisle4', 'Aisle5']
    statuses = ['Packaging', 'On Hold', 'Ready to Ship']
    weights = [6.3, 7.8, 9.1]
    i = 0

    while not rospy.is_shutdown():
        loc_msg = locations[i % len(locations)] + " (pub2)"
        stat_msg = statuses[i % len(statuses)] + " (pub2)"
        weight_msg = weights[i % len(weights)]

        pub_location.publish(loc_msg)
        pub_status.publish(stat_msg)
        pub_weight.publish(weight_msg)

        rospy.loginfo(f"[pub2] => Loc: {loc_msg}, Stat: {stat_msg}, Weight: {weight_msg}")
        i += 1
        rate.sleep()

if __name__ == '__main__':
    main()

