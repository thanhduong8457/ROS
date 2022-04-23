#include <iostream>
#include <string.h>
#include <vector>
#include <stdio.h>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "simu_visual/PositionArm.h"

#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#define pi 3.141592654

using namespace std;

serial::Serial ser;

typedef struct joint{
  string name;
  double position;
}joint_t;

vector<joint_t *> my_joint;

void init_joint()
{
  for(int i=0; i <= 11; i++)
  {
    joint_t *joint_temp = NULL;
    joint_temp = new joint_t;
    joint_temp->name = "";
    joint_temp->position = 0;

    my_joint.push_back(joint_temp);
  }
}

void show_infor()
{
  std_msgs::String data_send;

  float theta_1 = my_joint[0]->position*180/pi;
  float theta_2 = my_joint[1]->position*180/pi;
  float theta_3 = my_joint[2]->position*180/pi;

  data_send.data = "{\"theta1\":\"" + to_string(theta_1) +
                    "\",\"theta2\":\"" + to_string(theta_2) +
                    "\",\"theta3\":\"" + to_string(theta_3) + "\"}\r\n";

  ser.write(data_send.data);

  cout<<data_send.data;
}

void MoveGroupCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  for(int i=0; i<=11; i++)
  {
    my_joint[i]->name = msg->name[i];
    my_joint[i]->position = msg->position[i];
  }
  show_infor();
}

int main(int argc, char** argv)
{
  init_joint();
	ros::init(argc, argv, "simu_visual");

	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/joint_states", 1000, MoveGroupCallback);
  ros::Publisher chatter_pub = nh.advertise<simu_visual::PositionArm>("thanhduong", 1000);

  try
  {
    ser.setPort("/dev/ttyS8");
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("Unable to open port ");
    return -1;
  }

  if(ser.isOpen()) ROS_INFO_STREAM("Serial Port initialized");
  else return -1;

  ros::Rate loop_rate(50);

  simu_visual::PositionArm PositionArm;

  while (ros::ok())
  {
    PositionArm.base_brazo1 = my_joint[0]->position;
    PositionArm.base_brazo2 = my_joint[1]->position;
    PositionArm.base_brazo3 = my_joint[2]->position;

    PositionArm.codo1_a = my_joint[3]->position;
    PositionArm.codo1_b = my_joint[4]->position;

    PositionArm.codo2_a = my_joint[5]->position;
    PositionArm.codo2_b = my_joint[6]->position;

    PositionArm.codo3_a = my_joint[7]->position;
    PositionArm.codo3_b = my_joint[8]->position;

    PositionArm.act_x = my_joint[9]->position;
    PositionArm.act_y = my_joint[10]->position;
    PositionArm.act_z = my_joint[11]->position;

    chatter_pub.publish(PositionArm);

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  //ros::spin();
	return 0;
}



