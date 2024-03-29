#include <iostream>
#include <string.h>
#include <vector>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "my_delta_robot/image_pos.h"
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
bool is_send_status_to_image_node;

void add_point(double x, double y, double z, int type);
void status_delta_callback(const std_msgs::String::ConstPtr& msg);
void data_image_callback(const my_delta_robot::image_pos::ConstPtr& msg);

int main(int argc, char **argv) {
    ros::init(argc, argv, "node_a");
    ros::NodeHandle nh;

    ros::Subscriber data_image = nh.subscribe("data_image", 1000, data_image_callback);
    ros::Subscriber sub_status_delta = nh.subscribe("status_to_node_a", 1000, status_delta_callback);

    ros::Publisher chatter_pub = nh.advertise<my_delta_robot::posicionxyz>("send_to_node_b", 1000);
    ros::Publisher status_to_image_node = nh.advertise<std_msgs::String>("status_to_image_node", 1000);

    ros::Rate loop_rate(1);

    my_delta_robot::posicionxyz posicionxyz;
    std_msgs::String msg;

    status = false;
    is_send_status_to_image_node =  false;

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

        if(is_send_status_to_image_node)
        {
            msg.data = "Done";
            status_to_image_node.publish(msg);
            is_send_status_to_image_node = false;
        }

        ros::spinOnce();
    }

    return 0;
}

void add_point(double x, double y, double z, int type) {
    point_t *data = NULL;
    data = new point_t;

    data->position_x = x;
    data->position_y = y;
    data->position_z = z;
    data->type = type;

    my_point.push_back(data);
}

void status_delta_callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("status: [%s]", msg->data.c_str());

    if(my_point.size()!=0) {
        status = true;
    }
    if(my_point.size()==0) {
        is_send_status_to_image_node = true;
    }
}

void data_image_callback(const my_delta_robot::image_pos::ConstPtr& msg) {
    ROS_INFO("receive status from camera");
    for(int i = 0; i<msg->x.size(); i++) {
        add_point(msg->x[i], msg->y[i], msg->z[i], msg->type[i]);
        cout<<"point: "<<msg->x[i]<<" "<<msg->y[i]<<" "<<msg->z[i]<<" "<<msg->type[i]<<" has been added."<<endl;
    }
    status  = true;
}
