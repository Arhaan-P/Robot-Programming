#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32

def main():
    rospy.init_node('publisher1')
    pub_location = rospy.Publisher('/location', String, queue_size=10)
    pub_status = rospy.Publisher('/status', String, queue_size=10)
    pub_weight = rospy.Publisher('/weight', Float32, queue_size=10)
    rate = rospy.Rate(1)

    locations = ['Aisle1', 'Aisle2', 'Aisle3']
    statuses = ['In transit', 'Delivered', 'Delayed']
    weights = [5.5, 7.2, 18.0]
    i = 0

    while not rospy.is_shutdown():
        loc_msg = locations[i % len(locations)] + " (pub1)"
        stat_msg = statuses[i % len(statuses)] + " (pub1)"
        weight_msg = weights[i % len(weights)]

        pub_location.publish(loc_msg)
        pub_status.publish(stat_msg)
        pub_weight.publish(weight_msg)

        rospy.loginfo(f"[pub1] => Loc: {loc_msg}, Stat: {stat_msg}, Wt: {weight_msg}")
        i += 1
        rate.sleep()

if __name__ == '__main__':
    main()

