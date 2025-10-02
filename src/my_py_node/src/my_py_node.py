#!/usr/bin/env python3

import rospy

def is_prime(n):
    if n < 2:
        return False
    for i in range(2, n):
        if n % i == 0:
            return False
    return True

rospy.init_node('prime_node')

for num in range(1, 101):
    if is_prime(num):
        print(num)

