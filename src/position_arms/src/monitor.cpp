#include <string.h>
#include <vector>
#include <stdio.h>
#include <iostream>

#include "ros/ros.h"
#include "moveit_msgs/MotionPlanRequest.h"

typedef struct joint{
  std::string name;
  double position;
  double tolerance_above;
  double tolerance_below;
  double weight;
}joint_t;

std::vector<joint_t *> my_joint;

void init_joint()
{
  for(int i=0; i <= 6; i++){
    joint_t *joint_temp = NULL;
    joint_temp = new joint_t;
    joint_temp->name = "";
    joint_temp->position = 0;
    joint_temp->tolerance_above = 0;
    joint_temp->tolerance_below = 0;
    joint_temp->weight = 0;

    my_joint.push_back(joint_temp);
  }
}

void show_infor(){
  for(int i=0; i<=6; i++){
    ROS_INFO("%s : %f tolerance_above : %f tolerance_below : %f weight : %f", 
      my_joint[i]->name.c_str(), 
      my_joint[i]->position, 
      my_joint[i]->tolerance_above, 
      my_joint[i]->tolerance_below, 
      my_joint[i]->weight);
  }
}

void MoveGroupCallback(const moveit_msgs::MotionPlanRequest::ConstPtr& msg)
{
  for(int i=0; i<=6; i++){
    my_joint[i]->name = msg->goal_constraints[0].joint_constraints[i].joint_name;
    my_joint[i]->position = msg->goal_constraints[0].joint_constraints[i].position;
    my_joint[i]->tolerance_above = msg->goal_constraints[0].joint_constraints[i].tolerance_above;
    my_joint[i]->tolerance_below = msg->goal_constraints[0].joint_constraints[i].tolerance_below;
    my_joint[i]->weight = msg->goal_constraints[0].joint_constraints[i].weight;
  }
  show_infor();
  std::cout<<"--------------------------------------"<<std::endl;
}

int main(int argc, char** argv)
{
  init_joint();
	ros::init(argc, argv, "position_arms");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("move_group/motion_plan_request", 1000, MoveGroupCallback);
	ros::spin();
 
	return 0;

}

