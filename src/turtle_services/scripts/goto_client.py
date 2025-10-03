#!/usr/bin/env python3

import rospy
from turtle_services.srv import GoToPoint
import sys

def go_to_point_client(x, y):
    rospy.wait_for_service('go_to_point')
    try:
        go_to_point = rospy.ServiceProxy('go_to_point', GoToPoint)
        response = go_to_point(x, y)
        rospy.loginfo(f"Success: {response.success}, Message: {response.message}")
        return response
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None

if __name__ == '__main__':
    if len(sys.argv) == 3:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
    else:
        print("Usage: goto_client.py X Y")
        sys.exit(1)
    
    rospy.init_node('go_to_point_client')
    go_to_point_client(x, y)
