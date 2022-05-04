#include <iostream>
#include <string.h>
#include <vector>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "simu_visual/linear_speed_xyz.h"
#include "simu_visual/posicionxyz.h"

using namespace std;

typedef struct point{
  double position_x;
  double position_y;
  double position_z;
}point_t;

vector<point_t *> my_point;
bool status;
bool is_send_status_to_node_a;

double x_current, y_current, z_current;
double xx, yy, zz;
double x_fixed, y_fixed, z_fixed;

void add_point(double x, double y, double z);
void Status_Delta_Callback(const std_msgs::String::ConstPtr& msg);
void node_a_callback(const simu_visual::posicionxyz::ConstPtr& msg);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_b");

    ros::NodeHandle nh;

    ros::Subscriber receive_node_a = nh.subscribe("send_to_node_b", 1000, node_a_callback);
    ros::Subscriber sub_status_delta = nh.subscribe("status_delta", 1000, Status_Delta_Callback);

    ros::Publisher chatter_pub = nh.advertise<simu_visual::linear_speed_xyz>("input_ls_final", 1000);
    ros::Publisher status_to_node_a = nh.advertise<std_msgs::String>("status_to_node_a", 1000);

    ros::Rate loop_rate(1);

    simu_visual::linear_speed_xyz linear_speed_xyz;
    std_msgs::String msg;

    x_current = 0.0;
    y_current = 0.0;
    z_current = -375.0;

    x_fixed = 100.0;
    y_fixed = 100.0;
    z_fixed = -420.0;

    status = false;
    is_send_status_to_node_a = false;

    while (ros::ok())
    {
        if(status)
        {
            linear_speed_xyz.xo = my_point[0]->position_x;
            linear_speed_xyz.yo = my_point[0]->position_y;
            linear_speed_xyz.zo = my_point[0]->position_z;

            linear_speed_xyz.xf = my_point[1]->position_x;
            linear_speed_xyz.yf = my_point[1]->position_y;
            linear_speed_xyz.zf = my_point[1]->position_z;

            linear_speed_xyz.vmax = 2000.0;
            linear_speed_xyz.amax = 100000.0;

            loop_rate.sleep();
            chatter_pub.publish(linear_speed_xyz);

            cout<<"move from: "<<my_point[0]->position_x<<" "<<my_point[0]->position_y<<" "<<my_point[0]->position_z<<" to: "<<my_point[1]->position_x<<" "<<my_point[1]->position_y<<" "<<my_point[1]->position_z<<endl;

            delete [] my_point[0];
            my_point.erase(my_point.begin());

            status = false;
            ros::spinOnce();
        }

        if(is_send_status_to_node_a)
        {
            msg.data = "Point [" + to_string(xx) + " " + to_string(yy) + " " + to_string(zz) + "] is finished";
            status_to_node_a.publish(msg);
            
            is_send_status_to_node_a = false;
        }


        ros::spinOnce();
    }

    //ros::spin();
    return 0;
}

void add_point(double x, double y, double z)
{
    point_t *data = NULL;
    data = new point_t;

    data->position_x = x;
    data->position_y = y;
    data->position_z = z;

    my_point.push_back(data);
}

void Status_Delta_Callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("status from node_c: [%s]", msg->data.c_str());

    if(my_point.size()==1)
    {
        x_current = my_point[0]->position_x;
        y_current = my_point[0]->position_y;
        z_current = my_point[0]->position_z;

        my_point.clear();

        //cout<<"x_current: "<<x_current<<" y_current: "<<y_current<<" z_current: "<<z_current<<endl;

        is_send_status_to_node_a = true;

        cout<<endl;

    }

    if(my_point.size()!=0)  status = true;

}

void node_a_callback(const simu_visual::posicionxyz::ConstPtr& msg)
{
    xx = msg->x0;
    yy = msg->y0;
    zz = msg->z0;

    //cout<<"x: "<<xx<<" y: "<<yy<<" z: "<<zz<<endl;

    add_point(x_current, y_current, z_current);

    add_point(xx, yy, zz);
    add_point(xx, yy, zz-30);
    add_point(xx, yy, zz);

    add_point(x_fixed, x_fixed, z_fixed);
    add_point(x_fixed, x_fixed, z_fixed-30);
    add_point(x_fixed, x_fixed, z_fixed);

    status = true;
}
