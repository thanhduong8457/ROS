#include <iostream>
#include <string.h>
#include <vector>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "my_delta_robot/linear_speed_xyz.h"
#include "my_delta_robot/posicionxyz.h"
#include "my_delta_robot/vmax_amax.h"

using namespace std;

typedef struct point{
  double position_x;
  double position_y;
  double position_z;
  int gripper;
}point_t;

enum{
    circle = 0,
    square,
    triangle
};

vector<point_t *> my_point;
bool status;
bool is_send_status_to_node_a;

double vmax, amax;

double x_current, y_current, z_current, z_start, z_end;
double xx, yy, zz;
double x_circle, y_circle, z_circle;
double x_square, y_square, z_square;
double x_triangle, y_triangle, z_triangle;

void add_point(double x, double y, double z);
void Status_Delta_Callback(const std_msgs::String::ConstPtr& msg);
void node_a_callback(const my_delta_robot::posicionxyz::ConstPtr& msg);
void set_vmax_amax_callback(const my_delta_robot::vmax_amax::ConstPtr& msg);
void set_current_point_callback(const my_delta_robot::posicionxyz::ConstPtr& msg);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_b");

    ros::NodeHandle nh;

    ros::Subscriber receive_node_a = nh.subscribe("send_to_node_b", 1000, node_a_callback);
    ros::Subscriber set_vmax_amax = nh.subscribe("set_vmax_amax", 1000, set_vmax_amax_callback);
    ros::Subscriber set_current_point = nh.subscribe("set_current_point", 1000, set_current_point_callback);
    ros::Subscriber sub_status_delta = nh.subscribe("status_delta", 1000, Status_Delta_Callback);

    ros::Publisher chatter_pub = nh.advertise<my_delta_robot::linear_speed_xyz>("input_ls_final", 1000);
    ros::Publisher status_to_node_a = nh.advertise<std_msgs::String>("status_to_node_a", 1000);

    ros::Rate loop_rate(1);

    my_delta_robot::linear_speed_xyz linear_speed_xyz;
    std_msgs::String msg;

    //setting default value
    x_current = 0.0;
    y_current = 0.0;
    z_current = -375.0;

    x_circle = -100.0;
    y_circle = -100.0;
    z_circle = -453.0;

    x_square = 0.0;
    y_square = -100.0;
    z_square = -453.0;

    x_triangle = 100.0;
    y_triangle = -100.0;
    z_triangle = -453.0;

    z_start = 20;
    z_end = 10;

    vmax = 10000.0;
    amax = 100000.0;

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

            linear_speed_xyz.vmax = vmax;
            linear_speed_xyz.amax = amax;

            linear_speed_xyz.gripper = my_point[0]->gripper;

            loop_rate.sleep();
            chatter_pub.publish(linear_speed_xyz);

            cout<<"move from: "<<my_point[0]->position_x<<" "<<my_point[0]->position_y<<" "<<my_point[0]->position_z<<" to: "<<my_point[1]->position_x<<" "<<my_point[1]->position_y<<" "<<my_point[1]->position_z<<endl;

            delete [] my_point[0];
            my_point.erase(my_point.begin());

            status = false;
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

void add_point(double x, double y, double z, int gripper)
{
    point_t *data = NULL;
    data = new point_t;

    data->position_x = x;
    data->position_y = y;
    data->position_z = z;
    data->gripper = gripper;

    my_point.push_back(data);
}

void Status_Delta_Callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("status from main_node: [%s]", msg->data.c_str());

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

void node_a_callback(const my_delta_robot::posicionxyz::ConstPtr& msg)
{
    xx = msg->x0;
    yy = msg->y0;
    zz = msg->z0;
    int type = msg->type;

    cout<<"Processing point x: "<<xx<<" y: "<<yy<<" z: "<<zz<<endl;

    add_point(x_current, y_current, z_current, 0);

    add_point(xx, yy, zz, 0);
    add_point(xx, yy, zz - z_start, 1);
    add_point(xx, yy, zz, 1);

    switch (type){
    case circle:
        add_point(x_circle, y_circle, z_circle, 1);
        add_point(x_circle, y_circle, z_circle-z_end, 0);
        add_point(x_circle, y_circle, z_circle, 0);
        break;

    case square:
        add_point(x_square, y_square, z_square, 1);
        add_point(x_square, y_square, z_square-z_end, 0);
        add_point(x_square, y_square, z_square, 0);
        break;

    case triangle:
        add_point(x_triangle, y_triangle, z_triangle, 1);
        add_point(x_triangle, y_triangle, z_triangle-z_end, 0);
        add_point(x_triangle, y_triangle, z_triangle, 0);
        break;
    
    default:
        break;
    }
    
    status = true;
}

void set_vmax_amax_callback(const my_delta_robot::vmax_amax::ConstPtr& msg)
{
    vmax = msg->vmax;
    amax = msg->amax;
    cout<<"set vmax = "<<vmax<<", and set amax = "<<amax<<endl;
}

void set_current_point_callback(const my_delta_robot::posicionxyz::ConstPtr& msg)
{
    int temp = msg->type;

    switch (temp)
    {
    case (-1):
        x_current = msg->x0;
        y_current = msg->y0;
        z_current = msg->z0;

        if(z_current>-375||z_current<-480)
        {
            z_current = -375.0;
            cout<<"[ERROR] Invalid point, current point now set to x: "<<x_current<<" y: "<<y_current<<" z: "<<z_current<<endl;
        }
        else
        {
            cout<<"current point set to point x: "<<x_current<<" y: "<<y_current<<" z: "<<z_current<<endl;
        }
        break;
    
    case (0):
        x_circle = msg->x0;
        y_circle = msg->y0;
        z_circle = msg->z0;
        cout<<"Position to put CIRCLE is set to x: "<<x_circle<<" y: "<<y_circle<<" z: "<<z_circle<<endl;
        break;

    case (1):
        x_square = msg->x0;
        y_square = msg->y0;
        z_square = msg->z0;
        cout<<"Position to put SQUARE is set to x: "<<x_square<<" y: "<<y_square<<" z: "<<z_square<<endl;
        break;

    case (2):
        x_triangle = msg->x0;
        y_triangle = msg->y0;
        z_triangle = msg->z0;
        cout<<"Position to put TRIANGLE is set to x: "<<x_triangle<<" y: "<<y_triangle<<" z: "<<z_triangle<<endl;
        break;

    case (3):
        z_start = msg->x0;
        cout<<"the distance to go down and grip when START is set to: "<<z_start<<endl;
        break;

    case (4):
        z_end = msg->x0;
        cout<<"the distance to go down and grip when END is set to: "<<z_end<<endl;
        break;

    case (5):
        z_start = msg->x0;
        z_end = msg->y0;
        cout<<"the distance to go down and grip when START is set to: "<<z_start<<endl;
        cout<<"the distance to go down and grip when END is set to: "<<z_end<<endl;
        break;
    
    default:
        break;
    }
}
