#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "my_delta_robot/msg/linear_speed_xyz.hpp"
#include "my_delta_robot/msg/num_point.hpp"
#include "my_delta_robot/msg/vmax_amax.hpp"

#include "delta_robot.h"

class MyNode: public rclcpp::Node {
    int gripper;
    delta_robot *m_delta_robot;

    double position_value[13];

    rclcpp::Rate loop_rate;
    sensor_msgs::msg::JointState JointState;
    my_delta_robot::msg::VmaxAmax vm_am;
    std_msgs::msg::String msg;

    std::shared_ptr<rclcpp::Subscription<my_delta_robot::msg::LinearSpeedXYZ>> receive_node_a;
    std::shared_ptr<rclcpp::Subscription<my_delta_robot::msg::NumPoint>> set_num_point;
    std::shared_ptr<rclcpp::Subscription<my_delta_robot::msg::VmaxAmax>> set_vmax_amax;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> pub_for_rviz;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> status_to_node_b;
    std::shared_ptr<rclcpp::Publisher<my_delta_robot::msg::VmaxAmax>> v_a_out;

public:
    MyNode():Node("main_node"), loop_rate(7.8125) {
        RCLCPP_INFO(this->get_logger(), "main_node is created");

        receive_node_a = this->create_subscription<my_delta_robot::msg::LinearSpeedXYZ>("input_ls_final", 10, std::bind(&MyNode::callback_linear_speed_xyz, this, std::placeholders::_1));
        set_num_point = this->create_subscription<my_delta_robot::msg::NumPoint>("set_num_point", 10, std::bind(&MyNode::set_num_point_callback, this, std::placeholders::_1));
        set_vmax_amax = this->create_subscription<my_delta_robot::msg::VmaxAmax>("set_vmax_amax", 10, std::bind(&MyNode::set_vmax_amax_callback, this, std::placeholders::_1));

        pub_for_rviz = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        status_to_node_b = this->create_publisher<std_msgs::msg::String>("status_delta", 10);
        v_a_out = this->create_publisher<my_delta_robot::msg::VmaxAmax>("v_a_out", 10);

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

        m_delta_robot = new delta_robot(); // construct a new delta_robot
        m_delta_robot->set_vmax_amax(1500, 200000); // set vmax and amax
        m_delta_robot->set_resolution(120);
        // Create a thread to call main_func
        // std::thread(&MyNode::main_func, this).detach();
    }

    ~MyNode() {
        delete m_delta_robot;
    }
private:
    void callback_linear_speed_xyz(const my_delta_robot::msg::LinearSpeedXYZ::SharedPtr msg) {
        m_delta_robot->mStartPoint.x = msg->xo;
        m_delta_robot->mStartPoint.y = msg->yo;
        m_delta_robot->mStartPoint.z = msg->zo;

        m_delta_robot->mEndPoint.x = msg->xf;
        m_delta_robot->mEndPoint.y = msg->yf;
        m_delta_robot->mEndPoint.z = msg->zf;

        gripper = msg->gripper;

        if (m_delta_robot->mStartPoint != m_delta_robot->mEndPoint) {
            main_func();
        }
        else {
            std::cout << "Start point and end Point is duplicate" << std::endl;
        }
    }

    /// @brief 
    /// @param msg 
    void set_num_point_callback(const my_delta_robot::msg::NumPoint::SharedPtr msg) {
        if (msg->resolution > 0) {
            m_delta_robot->set_resolution(msg->resolution);
            std::cout << "set resolution = " << msg->resolution << std::endl;
        }
        else {
            std::cout << "ERROR to set resolution" << std::endl;
        }
    }
    
    /// @brief 
    void main_func() {
        double delta = 0;
        if (rclcpp::ok()) {
            m_delta_robot->system_linear();
            m_delta_robot->TrapezoidalVelocityProfile(); // Trapezoidal velocity profile
            m_delta_robot->system_linear_matrix();

            RCLCPP_INFO(this->get_logger(), "Creating Linear Path RVIZ!");

            // publish data of velocity and acceleration to visualize on rpt_plot
            vm_am.vmax = m_delta_robot->m_data_delta[0]->vel;
            vm_am.amax = m_delta_robot->m_data_delta[0]->acel;
            v_a_out->publish(vm_am);

            for (unsigned int i = 1; i < m_delta_robot->m_data_delta.size(); i++) {
                m_delta_robot->m_data_delta[i]->theta_val = m_delta_robot->inverse(m_delta_robot->m_data_delta[i]->position_val);
                m_delta_robot->CreateJointStateList(m_delta_robot->m_data_delta[i]->position_val,m_delta_robot->m_data_delta[i]->theta_val,
                    position_value);

                delta = m_delta_robot->m_data_delta[i]->time_point * 10 - m_delta_robot->m_data_delta[i - 1]->time_point * 10; // bug index out range

                // publish data of joints state to topic /joint_states
                for (int j = 0; j < 12; j++) {
                    JointState.position[j] = position_value[j];
                }

                JointState.position[12] = gripper;
                JointState.header.stamp = this->now();
                pub_for_rviz->publish(JointState);

                // publish data of velocity and acceleration to visualize on rpt_plot
                vm_am.vmax = m_delta_robot->m_data_delta[i]->vel;
                vm_am.amax = m_delta_robot->m_data_delta[i]->acel;
                v_a_out->publish(vm_am);

                rclcpp::Rate rate(1.0 / delta);
                rate.sleep();
            }

            // dump status
            msg.data = "(" + std::to_string(m_delta_robot->mStartPoint.x) + ", " 
                        + std::to_string(m_delta_robot->mStartPoint.y) + ", " 
                        + std::to_string(m_delta_robot->mStartPoint.z) + ") -> (" 
                        + std::to_string(m_delta_robot->mEndPoint.x) + ", " 
                        + std::to_string(m_delta_robot->mEndPoint.y) + ", " 
                        + std::to_string(m_delta_robot->mEndPoint.z) + ") DONE";
            status_to_node_b->publish(msg);
        }
    }

    /// @brief 
    /// @param msg 
    void set_vmax_amax_callback(const my_delta_robot::msg::VmaxAmax::SharedPtr msg) {
        m_delta_robot->set_vmax_amax(msg->vmax, msg->amax);
        cout << "set vmax = " << msg->vmax << ", and set amax = " << msg->amax << endl;
    }
};

/// @brief 
/// @param argc 
/// @param argv 
/// @return 
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
