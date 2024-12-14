#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "my_delta_robot/msg/image_pos.hpp"
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
bool is_send_status_to_image_node;

void add_point(double x, double y, double z, int type);
void status_delta_callback(const std_msgs::msg::String::SharedPtr msg);
void data_image_callback(const my_delta_robot::msg::ImagePos::SharedPtr msg);

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("node_a");

    auto data_image = node->create_subscription<my_delta_robot::msg::ImagePos>(
        "data_image", 10, data_image_callback);
    auto sub_status_delta = node->create_subscription<std_msgs::msg::String>(
        "status_to_node_a", 10, status_delta_callback);

    auto chatter_pub = node->create_publisher<my_delta_robot::msg::Posicionxyz>("send_to_node_b", 10);
    auto status_to_image_node = node->create_publisher<std_msgs::msg::String>("status_to_image_node", 10);

    rclcpp::Rate loop_rate(1);

    my_delta_robot::msg::Posicionxyz posicionxyz;
    std_msgs::msg::String msg;

    status = false;
    is_send_status_to_image_node = false;

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

        if (is_send_status_to_image_node) {
            msg.data = "Done";
            status_to_image_node->publish(msg);
            is_send_status_to_image_node = false;
        }

        rclcpp::spin_some(node);
    }

    return 0;
}

void add_point(double x, double y, double z, int type) {
    point_t *data = new point_t;

    data->position_x = x;
    data->position_y = y;
    data->position_z = z;
    data->type = type;

    my_point.push_back(data);
}

void status_delta_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "status: [%s]", msg->data.c_str());

    if (!my_point.empty()) {
        status = true;
    }
    if (my_point.empty()) {
        is_send_status_to_image_node = true;
    }
}

void data_image_callback(const my_delta_robot::msg::ImagePos::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "receive status from camera");
    for (size_t i = 0; i < msg->x.size(); i++) {
        add_point(msg->x[i], msg->y[i], msg->z[i], msg->type[i]);
        cout << "point: " << msg->x[i] << " " << msg->y[i] << " " << msg->z[i] << " " << msg->type[i] << " has been added." << endl;
    }
    status = true;
}