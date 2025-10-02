#include "ros/ros.h"
#include "std_msgs/String.h"

void radioCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received on C++ node: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "radio_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/news_radio", 10, radioCallback);
    ros::spin();
    return 0;
}

