#include <ros/ros.h>
#include <iostream>
#include <cmath>

bool isArmstrong(int n) {
    int sum = 0, temp = n, digits = 0;
    while (temp) {
        digits++;
        temp /= 10;
    }
    temp = n;
    while (temp) {
        sum += std::pow(temp % 10, digits);
        temp /= 10;
    }
    return sum == n;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "armstrong_node");
    ros::NodeHandle nh;
    for (int i = 1; i <= 1000; ++i) {
        if (isArmstrong(i)) {
            std::cout << i << std::endl;
        }
    }
    return 0;
}