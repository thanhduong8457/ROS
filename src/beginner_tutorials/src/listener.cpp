// #include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/String.h"
#include "std_msgs/msg/string.hpp"

void chatterCallback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    // ros::init(argc, argv, "listener");
    // ros::NodeHandle n;
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("listener");

    ros::Subscriber sub = n.subscribe("thanhduong", 1000, chatterCallback);
    ros::spin();

    return 0;
}