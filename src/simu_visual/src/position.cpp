#include <string.h>
#include <vector>
#include <stdio.h>
#include <iostream>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "simu_visual/PositionArm.h"

using namespace std;

typedef struct joint{
  std::string name;
  double position;
}joint_t;

std::vector<joint_t *> my_joint;

void init_joint()
{
  for(int i=0; i <= 11; i++){
    joint_t *joint_temp = NULL;
    joint_temp = new joint_t;
    joint_temp->name = "";
    joint_temp->position = 0;

    my_joint.push_back(joint_temp);
  }
}

void show_infor()
{
  for(int i=0; i<=11; i++){
    //ROS_INFO("%s : %f", my_joint[i]->name.c_str(), my_joint[i]->position);
    std::cout<<my_joint[i]->name<<":"<<my_joint[i]->position<<" ";
    //std::cout<<my_joint[i]->position<<" ";
  }
  std::cout<<std::endl;
}

void MoveGroupCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  for(int i=0; i<=11; i++)
  {
    my_joint[i]->name = msg->name[i];
    my_joint[i]->position = msg->position[i];
  }
  show_infor();
  //cout<<"--------------------------------------"<<endl;
}

int main(int argc, char** argv)
{
  init_joint();
	ros::init(argc, argv, "simu_visual");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/joint_states", 1000, MoveGroupCallback);
  ros::Publisher chatter_pub = nh.advertise<simu_visual::PositionArm>("thanhduong", 1000);
  ros::Rate loop_rate(50);

  while (ros::ok())
  {
    simu_visual::PositionArm PositionArm;

    PositionArm.joint_1 = my_joint[0]->position;
    PositionArm.joint_2 = my_joint[1]->position;
    PositionArm.joint_3 = my_joint[2]->position;
    PositionArm.joint_4 = my_joint[3]->position;
    PositionArm.joint_5 = my_joint[4]->position;
    PositionArm.joint_6 = my_joint[5]->position;
    PositionArm.joint_7 = my_joint[6]->position;
    PositionArm.joint_8 = my_joint[7]->position;
    PositionArm.joint_9 = my_joint[8]->position;
    PositionArm.joint_10 = my_joint[9]->position;
    PositionArm.joint_11 = my_joint[10]->position;
    PositionArm.joint_12 = my_joint[11]->position;

    chatter_pub.publish(PositionArm);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  //ros::spin();
	return 0;
}



