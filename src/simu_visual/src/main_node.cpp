#include "ros/ros.h"
#include "std_msgs/String.h"
#include "simu_visual/linear_speed_xyz.h"
#include "sensor_msgs/JointState.h"

#include "delta_define.h"
#include "delta_robot.h"

double call_xo_2, call_yo_2, call_zo_2;
double call_xf_2, call_yf_2, call_zf_2;
double call_vmax_2, call_amax_2;
int gripper;
bool status;

void callback_linear_speed_xyz(const simu_visual::linear_speed_xyz::ConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_node");

    ros::NodeHandle nh;

    // ##################  Subscriber  ##################
    ros::Subscriber receive_node_a = nh.subscribe("input_ls_final", 1000, callback_linear_speed_xyz);

    // ##################  Publisher  ##################
    ros::Publisher pub_for_rviz = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);
    ros::Publisher status_to_node_b = nh.advertise<std_msgs::String>("status_delta", 1000);

    ros::Rate loop_rate(7.8125);

    double dis, theta_y, theta_z;

    double rot_z[3][3] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    double rot_y[3][3] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    double rot_tras[4][4] ={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    double position_value[13] = {};

    std_msgs::String msg;
    sensor_msgs::JointState JointState;

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

    while (ros::ok())
    {
        if(status)
        {
	        m_delta_robot = new delta_robot;
            ROS_INFO("v_max = %lf, a_max = %lf", call_vmax_2, call_amax_2);
            //path_linear_speed(xx, yy, zz, x, y, z);
            m_delta_robot->system_linear(call_xo_2, call_yo_2, call_zo_2, call_xf_2, call_yf_2, call_zf_2, dis, rot_z, rot_y, theta_y, theta_z, rot_tras);

            // Trapezoidal velocity profile
            m_delta_robot->ls_v_a_total(0, dis, call_vmax_2, call_amax_2, 50, 150);

            // Reverse rotation end point, start point and trajectory #######
            m_delta_robot->system_linear_matrix(m_delta_robot->m_data_delta.size(), rot_z, rot_y, theta_y, theta_z, rot_tras);

            // // Inverse kinematics
            m_delta_robot->inverse_m();

            ROS_INFO("Creating Linear Path RVIZ!");

            m_delta_robot->angulos_eulerianos(1, 
                m_delta_robot->m_data_delta[0]->pos_x, m_delta_robot->m_data_delta[0]->pos_y, m_delta_robot->m_data_delta[0]->pos_z,
                m_delta_robot->m_data_delta[0]->theta_1, m_delta_robot->m_data_delta[0]->theta_2, m_delta_robot->m_data_delta[0]->theta_3, 
                gripper, position_value);
            
            double delta = 0.5;
            
            for(int i = 0; i<13; i++) JointState.position[i] = position_value[i];
            JointState.header.stamp = ros::Time::now(); 

            pub_for_rviz.publish(JointState);
            ros::Duration(delta).sleep();

            for (int i = 1; i < m_delta_robot->m_data_delta.size(); i++)
            {
                m_delta_robot->angulos_eulerianos(m_delta_robot->m_data_delta[i]->tiempo*10, 
                    m_delta_robot->m_data_delta[i]->pos_x, m_delta_robot->m_data_delta[i]->pos_y, m_delta_robot->m_data_delta[i]->pos_z,
                    m_delta_robot->m_data_delta[i]->theta_1, m_delta_robot->m_data_delta[i]->theta_2, m_delta_robot->m_data_delta[i]->theta_3, 
                    gripper, position_value);
                
                delta = m_delta_robot->m_data_delta[i]->tiempo * 10 - m_delta_robot->m_data_delta[i-1]->tiempo * 10;
                
                for(int i = 0; i<13; i++) JointState.position[i] = position_value[i];
                JointState.header.stamp = ros::Time::now();

                pub_for_rviz.publish(JointState);
                ros::Duration(delta).sleep();
            }

            msg.data = "from " + to_string(call_xo_2) + " " + to_string(call_yo_2) + " " + to_string(call_zo_2) + " to " + to_string(call_xf_2) + " " + to_string(call_yf_2) + " " + to_string(call_zf_2) + " is finished";
            status_to_node_b.publish(msg);

            delete m_delta_robot;

            status = false;
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

void callback_linear_speed_xyz(const simu_visual::linear_speed_xyz::ConstPtr& msg)
{
    call_xo_2 = msg->xo;
    call_yo_2 = msg->yo;
    call_zo_2 = msg->zo;

    call_xf_2 = msg->xf;
    call_yf_2 = msg->yf;
    call_zf_2 = msg->zf;

    call_vmax_2 = msg->vmax;
    call_amax_2 = msg->amax;

    gripper = msg->gripper;

    status = true;
}
