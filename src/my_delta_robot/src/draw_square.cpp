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

enum{
    CURRENT_POINT = -1,
    CIRCLE_POSITION = 0,
    SQUARE_POSITION,
    TRIANGLE,
    Z_START,
    Z_END,
    Z_START_END,
    DRAW_SQUARE
};

vector<point_t *> my_point;
bool status;
bool is_done;

double vmax, amax;

double x_current, y_current, z_current, z_start, z_end;

double x_A, y_A, z_A;
double x_B, y_B, z_B;
double x_C, y_C, z_C;
double x_D, y_D, z_D;

void add_point(double x, double y, double z);
void Status_Delta_Callback(const std_msgs::String::ConstPtr& msg);
void set_vmax_amax_callback(const my_delta_robot::vmax_amax::ConstPtr& msg);
void set_current_point_callback(const my_delta_robot::posicionxyz::ConstPtr& msg);
void draw_new_square();


int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_b");

    ros::NodeHandle nh;

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

    x_A = -100.0;
    y_A = -100.0;
    z_A = -453.0;

    x_B = 0.0;
    y_B = -100.0;
    z_B = -453.0;

    x_C = 100.0;
    y_C = -100.0;
    z_C = -453.0;

    x_D = 100.0;
    y_D = -100.0;
    z_D = -453.0;

    z_start = 20;
    z_end = 10;

    vmax = 10000.0;
    amax = 100000.0;

    draw_new_square();

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

        // if(is_done == true)
        // {
        //     draw_new_square();
        //     is_done = false;
        // }

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

        is_done = true;

        cout<<endl;

    }

    if(my_point.size()!=0)
    {
        status = true;
    }

}


void draw_new_square()
{
    add_point(x_current, y_current, z_current, 0);
    add_point(x_A, y_A, z_A - z_end, 0);
    add_point(x_B, y_B, z_B - z_end, 0);
    add_point(x_C, y_C, z_C - z_end, 0);
    add_point(x_D, y_D, z_D - z_end, 0);
    add_point(x_A, y_A, z_A - z_end, 0);
    add_point(x_current, y_current, z_current, 0);
    
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
    case (CURRENT_POINT):
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
    
    case (CIRCLE_POSITION):
        x_A = msg->x0;
        y_A = msg->y0;
        z_A = msg->z0;
        cout<<"Position to put CIRCLE is set to x: "<<x_A<<" y: "<<y_A<<" z: "<<z_A<<endl;
        break;

    case (SQUARE_POSITION):
        x_B = msg->x0;
        y_B = msg->y0;
        z_B = msg->z0;
        cout<<"Position to put SQUARE is set to x: "<<x_B<<" y: "<<y_B<<" z: "<<z_B<<endl;
        break;

    case (TRIANGLE):
        x_C = msg->x0;
        y_C = msg->y0;
        z_C = msg->z0;
        cout<<"Position to put TRIANGLE is set to x: "<<x_C<<" y: "<<y_C<<" z: "<<z_C<<endl;
        break;

    case (Z_START):
        z_start = msg->x0;
        cout<<"the distance to go down and grip when START is set to: "<<z_start<<endl;
        break;

    case (Z_END):
        z_end = msg->x0;
        cout<<"the distance to go down and grip when END is set to: "<<z_end<<endl;
        break;

    case (Z_START_END):
        z_start = msg->x0;
        z_end = msg->y0;
        cout<<"the distance to go down and grip when START is set to: "<<z_start<<endl;
        cout<<"the distance to go down and grip when END is set to: "<<z_end<<endl;
        break;
    
    case (DRAW_SQUARE):
        if(is_done == true)
        {
            draw_new_square();
            is_done = false;
        }
    
    default:
        break;
    }
}
