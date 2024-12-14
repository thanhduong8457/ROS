#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "my_delta_robot/msg/linear_speed_xyz.hpp"
#include "my_delta_robot/msg/num_point.hpp"
#include "my_delta_robot/msg/vmax_amax.hpp"

#include "delta_define.h"
#include "delta_robot.h"

Point point0;
Point pointf;

double call_vmax_2, call_amax_2;
int num_point_1, num_point_2;

int gripper;
bool status;

void callback_linear_speed_xyz(const my_delta_robot::msg::LinearSpeedXYZ::SharedPtr msg);
void set_num_point_callback(const my_delta_robot::msg::NumPoint::SharedPtr msg);

/// @brief 
/// @param argc 
/// @param argv 
/// @return 
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("main_node");

    // ##################  Subscriber  ##################
    auto receive_node_a = node->create_subscription<my_delta_robot::msg::LinearSpeedXYZ>(
        "input_ls_final", 10, callback_linear_speed_xyz);
    auto set_num_point = node->create_subscription<my_delta_robot::msg::NumPoint>(
        "set_num_point", 10, set_num_point_callback);

    // ##################  Publisher  ##################
    auto pub_for_rviz = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    auto status_to_node_b = node->create_publisher<std_msgs::msg::String>("status_delta", 1);
    auto v_a_out = node->create_publisher<my_delta_robot::msg::VmaxAmax>("v_a_out", 1);

    rclcpp::Rate loop_rate(7.8125);

    num_point_1 = 120;
    num_point_2 = 120;

    double dis, theta_y, theta_z;

    double rot_z[3][3] = {0};
    double rot_y[3][3] = {0};
    double rot_tras[4][4] = {0};
    double position_value[13] = {0};

    std_msgs::msg::String msg;
    sensor_msgs::msg::JointState JointState;
    my_delta_robot::msg::VmaxAmax vm_am;

    JointState.header.frame_id = "";

    JointState.name.resize(13);
    JointState.position.resize(13);

    JointState.name[0] = "base_brazo1";
    JointState.name[1] = "base_brazo2";
    JointState.name[2] = "base_brazo3";
    JointState.name[3] = "codo1_a";
    JointState.name[4] = "codo1_b";
    JointState.name[5] = "codo2_a";
    JointState.name[6] = "codo2_b";
    JointState.name[7] = "codo3_a";
    JointState.name[8] = "codo3_b";
    JointState.name[9] = "act_x";
    JointState.name[10] = "act_y";
    JointState.name[11] = "act_z";
    JointState.name[12] = "gripper";

    status = false;
    delta_robot *m_delta_robot = new delta_robot; // construct a new delta_robot
    m_delta_robot->vmax = call_vmax_2;
    m_delta_robot->amax = call_amax_2;

    while (rclcpp::ok()) {
        if (status) {
            RCLCPP_INFO(node->get_logger(), "v_max = %lf, a_max = %lf", m_delta_robot->vmax, m_delta_robot->amax);

            m_delta_robot->system_linear(point0, pointf, dis, rot_z, rot_y, theta_y, theta_z, rot_tras);
            m_delta_robot->trapezoidal_velocity_profile(0, dis, num_point_1, num_point_2); // Trapezoidal velocity profile

            // Reverse rotation end point, start point and trajectory #######
            m_delta_robot->system_linear_matrix(m_delta_robot->m_data_delta.size(), rot_z, rot_y, theta_y, theta_z, rot_tras);

            // Inverse kinematics
            m_delta_robot->inverse_m();

            RCLCPP_INFO(node->get_logger(), "Creating Linear Path RVIZ!");

            m_delta_robot->angulos_eulerianos(1,
                                              m_delta_robot->m_data_delta[0]->position_val,
                                              m_delta_robot->m_data_delta[0]->theta_val,
                                              gripper,
                                              position_value);

            double delta = 0.5;

            // publish data of joints state to topic /joint_states
            for (int i = 0; i < 13; i++)
                JointState.position[i] = position_value[i];
            JointState.header.stamp = node->now();
            pub_for_rviz->publish(JointState);

            // publish data of velocity and acceleration to visualize on rpt_plot
            vm_am.vmax = m_delta_robot->m_data_delta[0]->vel;
            vm_am.amax = m_delta_robot->m_data_delta[0]->acel;
            v_a_out->publish(vm_am);

            rclcpp::Rate rate(1.0 / delta);
            rate.sleep();

            for (size_t i = 1; i < m_delta_robot->m_data_delta.size(); i++) {
                m_delta_robot->angulos_eulerianos(
                    m_delta_robot->m_data_delta[i]->time_point * 10,
                    m_delta_robot->m_data_delta[i]->position_val,
                    m_delta_robot->m_data_delta[i]->theta_val,
                    gripper,
                    position_value);

                delta = m_delta_robot->m_data_delta[i]->time_point * 10 - m_delta_robot->m_data_delta[i - 1]->time_point * 10;

                // publish data of joints state to topic /joint_states
                for (int j = 0; j < 13; j++)
                    JointState.position[j] = position_value[j];
                JointState.header.stamp = node->now();
                pub_for_rviz->publish(JointState);

                // publish data of velocity and acceleration to visualize on rpt_plot
                vm_am.vmax = m_delta_robot->m_data_delta[i]->vel;
                vm_am.amax = m_delta_robot->m_data_delta[i]->acel;
                v_a_out->publish(vm_am);

                rclcpp::Rate rate(1.0 / delta);
                rate.sleep();
            }

            // dump status
            msg.data = "from " + std::to_string(point0.x) + " " + std::to_string(point0.y) + " " + std::to_string(point0.z) + " to " + std::to_string(pointf.x) + " " + std::to_string(pointf.y) + " " + std::to_string(pointf.z) + " is finished";
            status_to_node_b->publish(msg);

            status = false;
        }

        loop_rate.sleep();
        rclcpp::spin_some(node);
    }

    delete m_delta_robot;
    rclcpp::shutdown();
    return 0;
}

/// @brief 
/// @param msg 
void callback_linear_speed_xyz(const my_delta_robot::msg::LinearSpeedXYZ::SharedPtr msg) {
    point0.x = msg->xo;
    point0.y = msg->yo;
    point0.z = msg->zo;

    pointf.x = msg->xf;
    pointf.y = msg->yf;
    pointf.z = msg->zf;

    call_vmax_2 = msg->vmax;
    call_amax_2 = msg->amax;

    gripper = msg->gripper;

    if ((point0.x == pointf.x) && (point0.y == pointf.y) && (point0.z == pointf.z)) {
        std::cout << "Start point and end Point is duplicate" << std::endl;
        status = false;
    }
    else {
        status = true;
    }
}

/// @brief 
/// @param msg 
void set_num_point_callback(const my_delta_robot::msg::NumPoint::SharedPtr msg) {
    if (msg->num_point_1 > 0 && msg->num_point_2 > 0) {
        num_point_1 = msg->num_point_1;
        num_point_2 = msg->num_point_2;
        std::cout << "set num_point_1 = " << num_point_1 << " and num_point_2 = " << num_point_2 << std::endl;
    }
    else {
        std::cout << "ERROR to set num_point" << std::endl;
    }
}