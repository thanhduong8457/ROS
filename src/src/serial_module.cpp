#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "my_delta_robot/msg/position_arm.hpp"

#include <serial/serial.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>

#define pi 3.141592654

using namespace std;

serial::Serial ser;

typedef struct joint {
    string name;
    double position;
} joint_t;

vector<joint_t *> my_joint;

/// @brief 
void init_joint() {
    for (int i = 0; i <= 12; i++) {
        joint_t *joint_temp = new joint_t;
        joint_temp->name = "";
        joint_temp->position = 0;

        my_joint.push_back(joint_temp);
    }
}

/// @brief 
void show_infor() {
    std_msgs::msg::String data_send;

    float theta_1 = my_joint[0]->position * 180 / pi;
    float theta_2 = my_joint[1]->position * 180 / pi;
    float theta_3 = my_joint[2]->position * 180 / pi;

    int gripper = my_joint[12]->position;

    data_send.data = "{\"tt1\":\"" + to_string((int)(theta_1 * 1000)) +
                     "\",\"tt2\":\"" + to_string((int)(theta_2 * 1000)) +
                     "\",\"tt3\":\"" + to_string((int)(theta_3 * 1000)) + "\",\"gp\":\"" + to_string(gripper) + "\"}\n";

    ser.write(data_send.data);

    cout << data_send.data;
}

/// @brief 
/// @param msg 
void MoveGroupCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (int i = 0; i <= 12; i++) {
        my_joint[i]->name = msg->name[i];
        my_joint[i]->position = msg->position[i];
    }
    show_infor();
}

/// @brief 
/// @param argc 
/// @param argv 
/// @return 
int main(int argc, char **argv) {
    init_joint();
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("serial_module");

    auto sub = node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, MoveGroupCallback);
    auto chatter_pub = node->create_publisher<delta_robot::msg::PositionArm>("thanhduong", 10);

    try {
        ser.setPort("/dev/ttyTHS1");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e) {
        RCLCPP_ERROR(node->get_logger(), "Unable to open port ");
        return -1;
    }

    if (ser.isOpen())
        RCLCPP_INFO(node->get_logger(), "Serial Port initialized");
    else
        return -1;

    rclcpp::Rate loop_rate(50);

    delta_robot::msg::PositionArm PositionArm;

    while (rclcpp::ok()) {
        PositionArm.base_brazo1 = my_joint[0]->position;
        PositionArm.base_brazo2 = my_joint[1]->position;
        PositionArm.base_brazo3 = my_joint[2]->position;

        PositionArm.codo1_a = my_joint[3]->position;
        PositionArm.codo1_b = my_joint[4]->position;

        PositionArm.codo2_a = my_joint[5]->position;
        PositionArm.codo2_b = my_joint[6]->position;

        PositionArm.codo3_a = my_joint[7]->position;
        PositionArm.codo3_b = my_joint[8]->position;

        PositionArm.act_x = my_joint[9]->position;
        PositionArm.act_y = my_joint[10]->position;
        PositionArm.act_z = my_joint[11]->position;

        chatter_pub->publish(PositionArm);

        loop_rate.sleep();
        rclcpp::spin_some(node);
    }

    return 0;
}