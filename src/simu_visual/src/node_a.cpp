#include <iostream>
#include <string.h>
#include <vector>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "simu_visual/posicionxyz.h"

using namespace std;

typedef struct point{
  double position_x;
  double position_y;
  double position_z;
}point_t;

vector<point_t *> my_point_a;
bool status_a;

void add_point(double x, double y, double z)
{
    point_t *data = NULL;
    data = new point_t;

    data->position_x = x;
    data->position_y = y;
    data->position_z = z;

    my_point_a.push_back(data);
}

void Status_Delta_Callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("status: [%s]", msg->data.c_str());
    status_a = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_a");
    ros::NodeHandle nh;

    ros::Subscriber sub_status_delta = nh.subscribe("status_to_node_a", 1000, Status_Delta_Callback);
    ros::Publisher chatter_pub = nh.advertise<simu_visual::posicionxyz>("send_to_node_b", 1000);

    ros::Rate loop_rate(1);

    add_point(50.0, 0.0, -420.0);
    add_point(60.0, 100.0, -420.0);
    add_point(70.0, 100.0, -420.0);
    add_point(80.0, 100.0, -420.0);
    add_point(200.0, -100.0, -420.0);
    add_point(0.0, -100.0, -420.0);
    add_point(30.0, -100.0, -420.0);
    add_point(0.0, 0.0, -420.0);

    simu_visual::posicionxyz posicionxyz;
    status_a = true;

    while (ros::ok())
    {
        if(status_a)
        {
            posicionxyz.x0 = my_point_a[0]->position_x;
            posicionxyz.y0 = my_point_a[0]->position_y;
            posicionxyz.z0 = my_point_a[0]->position_z;

            loop_rate.sleep();
            chatter_pub.publish(posicionxyz);

            delete [] my_point_a[0];
            my_point_a.erase(my_point_a.begin());

            status_a = false;
            ros::spinOnce();
        }

        if(my_point_a.size()==1)
        {
            my_point_a.clear();
            break;
        }

        ros::spinOnce();
    }

    //ros::spin();
    return 0;
}
