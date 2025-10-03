#!/usr/bin/env python3

import rospy
from turtle_services.srv import AddTwoInts, AddTwoIntsResponse

def handle_add_two_ints(req):
    result = req.a + req.b
    rospy.loginfo(f"Request: {req.a} + {req.b} = {result}")
    return AddTwoIntsResponse(result)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    service = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    rospy.loginfo("Add Two Ints Service is ready!")
    rospy.spin()

if __name__ == '__main__':
    add_two_ints_server()
