#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "my_delta_robot/msg/posicionxyz.hpp"

using namespace std;

typedef struct point {
    double position_x;
    double position_y;
    double position_z;
    int type;
} point_t;

enum {
    circle = 0,
    square,
    triangle
};

vector<point_t *> my_point;
bool status;

/// @brief 
/// @param x 
/// @param y 
/// @param z 
/// @param type 
void add_point(double x, double y, double z, int type) {
    point_t *data = new point_t;
    data->position_x = x;
    data->position_y = y;
    data->position_z = z;
    data->type = type;

    my_point.push_back(data);
}

/// @brief 
/// @param msg 
void Status_Delta_Callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "status: [%s]", msg->data.c_str());

    if (!my_point.empty()) {
        status = true;
    }
}

/// @brief 
/// @param argc 
/// @param argv 
/// @return 
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("node_a");

    auto sub_status_delta = node->create_subscription<std_msgs::msg::String>("status_to_node_a", 10, Status_Delta_Callback);
    auto chatter_pub = node->create_publisher<delta_robot::msg::Posicionxyz>("send_to_node_b", 10);

    rclcpp::Rate loop_rate(1);

    for (int i = 0; i < 101; i++) {
        add_point(0.0, 100.0, -453.0, square);
    }

    delta_robot::msg::Posicionxyz posicionxyz;
    status = true;

    while (rclcpp::ok()) {
        if (status) {
            posicionxyz.x0 = my_point[0]->position_x;
            posicionxyz.y0 = my_point[0]->position_y;
            posicionxyz.z0 = my_point[0]->position_z;
            posicionxyz.type = my_point[0]->type;

            loop_rate.sleep();
            chatter_pub->publish(posicionxyz);

            delete my_point[0];
            my_point.erase(my_point.begin());

            status = false;
        }

        rclcpp::spin_some(node);
    }

    return 0;
}