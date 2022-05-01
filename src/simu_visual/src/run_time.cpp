#include <iostream>
#include <string.h>
#include <vector>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "simu_visual/linear_speed_xyz.h"

using namespace std;

typedef struct point{
  double position_x;
  double position_y;
  double position_z;
}point_t;

vector<point_t *> my_point;
bool status;

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
    ROS_INFO("status: [%s]", msg->data.c_str());
    status = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "run_time");
    ros::NodeHandle nh;
    ros::Subscriber sub_status_delta = nh.subscribe("status_delta", 1000, Status_Delta_Callback);
    ros::Publisher chatter_pub = nh.advertise<simu_visual::linear_speed_xyz>("input_ls_final", 1000);
    ros::Rate loop_rate(1);

    add_point(0.0, 0.0, -375);
    add_point(0.0, 150.0, -490.0);
    add_point(0.0, 0.0, -375.0);
    add_point(0.0, -150, -490.0);
    add_point(0.0, 0.0, -375.0);
    add_point(100.0, 100.0, -490.0);
    add_point(-100.0, -100.0, -490.0);
    add_point(0.0, 0.0, -490.0);
    add_point(0.0, 0.0, -375.0);

    simu_visual::linear_speed_xyz linear_speed_xyz;
    status = true;

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

            cout<<"move from: "<<my_point[0]->position_x<<" "<<my_point[0]->position_y<<" "<<my_point[0]->position_z<<endl;
            cout<<"move to: "<<my_point[1]->position_x<<" "<<my_point[1]->position_y<<" "<<my_point[1]->position_z<<endl;

            //my_point[0].clear();
            delete [] my_point[0];
            my_point.erase(my_point.begin());

            status = false;

            ros::spinOnce();
        }

        if(my_point.size()==1)
        {
            break;
        }

        ros::spinOnce();
    }

    //ros::spin();
    return 0;
}



