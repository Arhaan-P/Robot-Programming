#!/usr/bin/env python3

import rospy
from turtle_services.srv import AddTwoInts
import sys

def add_two_ints_client(a, b):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        response = add_two_ints(a, b)
        return response.sum
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None

if __name__ == '__main__':
    if len(sys.argv) == 3:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
    else:
        print("Usage: add_client.py A B")
        sys.exit(1)
    
    rospy.init_node('add_two_ints_client')
    
    result = add_two_ints_client(a, b)
    if result is not None:
        rospy.loginfo(f"Result: {a} + {b} = {result}")
