#include "ros/ros.h"
#include "std_msgs/String.h"
#include "my_delta_robot/linear_speed_xyz.h"
#include "my_delta_robot/num_point.h"
#include "sensor_msgs/JointState.h"
#include "my_delta_robot/vmax_amax.h"

#include "delta_define.h"
#include "delta_robot.h"

Point point0;
Point pointf;

double call_vmax_2, call_amax_2;

int num_point_1, num_point_2;

int gripper;
bool status;

void callback_linear_speed_xyz(const my_delta_robot::linear_speed_xyz::ConstPtr& msg);
void set_num_point_callback(const my_delta_robot::num_point::ConstPtr& msg);

int main(int argc, char **argv) {
    ros::init(argc, argv, "main_node");

    ros::NodeHandle nh;

    // ##################  Subscriber  ##################
    ros::Subscriber receive_node_a = nh.subscribe("input_ls_final", 1000, callback_linear_speed_xyz);
    ros::Subscriber set_num_point = nh.subscribe("set_num_point", 1000, set_num_point_callback);

    // ##################  Publisher  ##################
    ros::Publisher pub_for_rviz = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);
    ros::Publisher status_to_node_b = nh.advertise<std_msgs::String>("status_delta", 1000);
    ros::Publisher v_a_out = nh.advertise<my_delta_robot::vmax_amax>("v_a_out", 1000);

    ros::Rate loop_rate(7.8125);

    num_point_1 = 120;
    num_point_2 = 120;

    double dis, theta_y, theta_z;

    double rot_z[3][3] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    double rot_y[3][3] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    double rot_tras[4][4] ={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    double position_value[13] = {};

    std_msgs::String msg;
    sensor_msgs::JointState JointState;
    my_delta_robot::vmax_amax vm_am;

    JointState.header.frame_id = "";  

    JointState.name.resize(13);
    JointState.position.resize(13);

    //JointState.header = std_msgs::msg::Header();

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
    
    // JointState.velocity[] = {};
    // JointState.effort[] = {};

    status = false;
    delta_robot *m_delta_robot = NULL;
    m_delta_robot = new delta_robot; // contruct a new delta_robot
    m_delta_robot->vmax = call_vmax_2;
    m_delta_robot->amax = call_amax_2;

    while (ros::ok()) {
        if(status) {
            ROS_INFO("v_max = %lf, a_max = %lf", m_delta_robot->vmax, m_delta_robot->amax);

            m_delta_robot->system_linear(point0, pointf, dis, rot_z, rot_y, theta_y, theta_z, rot_tras);
            m_delta_robot->trapezoidal_velocity_profile(0, dis, num_point_1, num_point_2); // Trapezoidal velocity profile

            // Reverse rotation end point, start point and trajectory #######
            m_delta_robot->system_linear_matrix(m_delta_robot->m_data_delta.size(), rot_z, rot_y, theta_y, theta_z, rot_tras);

            // // Inverse kinematics
            m_delta_robot->inverse_m();

            ROS_INFO("Creating Linear Path RVIZ!");

            m_delta_robot->angulos_eulerianos(1, 
                m_delta_robot->m_data_delta[0]->position_val,
                m_delta_robot->m_data_delta[0]->theta_val,
                gripper, 
                position_value
            );
            
            double delta = 0.5;
            
            // publish data of joins state to topic /Joints_state
            for(int i = 0; i<13; i++) JointState.position[i] = position_value[i];
            JointState.header.stamp = ros::Time::now(); 
            pub_for_rviz.publish(JointState);

            // publish data of velocity and aceleration to visual on rpt_plot
            vm_am.vmax = m_delta_robot->m_data_delta[0]->vel;
            vm_am.amax = m_delta_robot->m_data_delta[0]->acel;
            v_a_out.publish(vm_am);

            ros::Duration(delta).sleep();

            for (int i = 1; i < m_delta_robot->m_data_delta.size(); i++) {
                m_delta_robot->angulos_eulerianos(
                    m_delta_robot->m_data_delta[i]->time_point*10, 
                    m_delta_robot->m_data_delta[i]->position_val,
                    m_delta_robot->m_data_delta[i]->theta_val,
                    gripper, 
                    position_value
                );
                
                delta = m_delta_robot->m_data_delta[i]->time_point * 10 - m_delta_robot->m_data_delta[i-1]->time_point * 10;
                
                // publish data of joins state to topic /Joints_state
                for(int i = 0; i<13; i++) JointState.position[i] = position_value[i];
                JointState.header.stamp = ros::Time::now();
                pub_for_rviz.publish(JointState);

                // publish data of velocity and aceleration to visual on rpt_plot
                vm_am.vmax = m_delta_robot->m_data_delta[i]->vel;
                vm_am.amax = m_delta_robot->m_data_delta[i]->acel;
                v_a_out.publish(vm_am);

                ros::Duration(delta).sleep();
            }

            // dump status
            msg.data = "from " + to_string(point0.x) + " " + to_string(point0.y) + " " + to_string(point0.z) + " to " + to_string(pointf.x) + " " + to_string(pointf.y) + " " + to_string(pointf.z) + " is finished";
            status_to_node_b.publish(msg);

            // delete contructor

            status = false;
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    delete m_delta_robot;
    return 0;
}

/// @brief 
/// @param msg 
void callback_linear_speed_xyz(const my_delta_robot::linear_speed_xyz::ConstPtr& msg) {
    point0.x = msg->xo;
    point0.y = msg->yo;
    point0.z = msg->zo;

    pointf.x = msg->xf;
    pointf.y = msg->yf;
    pointf.z = msg->zf;

    call_vmax_2 = msg->vmax;
    call_amax_2 = msg->amax;

    gripper = msg->gripper;

    if((point0.x == pointf.x) && (point0.y == pointf.y) && (point0.z == pointf.z)) {
        cout<<"Start point and end Point is duplicate"<<endl;
        status = false;
    }
    else {
        status = true;
    }

}

/// @brief 
/// @param msg 
void set_num_point_callback(const my_delta_robot::num_point::ConstPtr& msg) {
    if(msg->num_point_1>0 && msg->num_point_2>0) {
        num_point_1 = msg->num_point_1;
        num_point_2 = msg->num_point_2;

        cout<<"set num_point_1 = "<<num_point_1<<" and num_point_2 = "<<num_point_2<<endl;
    }
    else {
        cout<<"ERORR to set num_point"<<endl;
    }
}
