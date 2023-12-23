#include <iostream>
#include <string.h>
#include <vector>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "my_delta_robot/posicionxyz.h"

using namespace std;

typedef struct point{
  double position_x;
  double position_y;
  double position_z;
  int type;
}point_t;

enum{
    circle = 0,
    square,
    triangle
};

vector<point_t *> my_point;
bool status;
bool is_exit;

void add_point(double x, double y, double z, int type) {
    point_t *data = NULL;
    data = new point_t;

    data->position_x = x;
    data->position_y = y;
    data->position_z = z;
    data->type = type;

    my_point.push_back(data);
}

void Status_Delta_Callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("status: [%s]", msg->data.c_str());

    if(my_point.size()!=0)  status = true;

    if(my_point.size()==0) {
        is_exit = true;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "node_a");
    ros::NodeHandle nh;

    ros::Subscriber sub_status_delta = nh.subscribe("status_to_node_a", 1000, Status_Delta_Callback);
    ros::Publisher chatter_pub = nh.advertise<my_delta_robot::posicionxyz>("send_to_node_b", 1000);

    ros::Rate loop_rate(1);

    // add_point(0.0, 100.0, -453.0, circle);
    // add_point(0.0, 100.0, -453.0, square);
    // add_point(0.0, 100.0, -453.0, triangle);

    // add_point(0.0, 100.0, -453.0, circle);
    // add_point(0.0, 100.0, -453.0, square);
    // add_point(0.0, 100.0, -453.0, triangle);

    // add_point(0.0, 100.0, -453.0, circle);
    // add_point(0.0, 100.0, -453.0, square);
    // add_point(0.0, 100.0, -453.0, triangle);

    for(int i = 0; i < 101; i++) {
        add_point(0.0, 100.0, -453.0, square);
    }

    my_delta_robot::posicionxyz posicionxyz;
    status = true;
    is_exit =  false;

    while (ros::ok()) {
        if(status) {
            posicionxyz.x0 = my_point[0]->position_x;
            posicionxyz.y0 = my_point[0]->position_y;
            posicionxyz.z0 = my_point[0]->position_z;
            posicionxyz.type = my_point[0]->type;

            loop_rate.sleep();
            chatter_pub.publish(posicionxyz);

            delete [] my_point[0];
            my_point.erase(my_point.begin());

            status = false;
        }

        if(is_exit) {
            return 0;
        }

        ros::spinOnce();
    }

    return 0;
}
