#include "ros/ros.h"
#include "beginner_tutorials/AddThreeInts.h"

bool add(beginner_tutorials::AddThreeInts::Request  &req,
         beginner_tutorials::AddThreeInts::Response &res)
{
  res.sum = req.a + req.b + req.c;
  ROS_INFO("request: x=%ld, y=%ld, z=%ld", (long int)req.a, (long int)req.b, (long int)req.c);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_numbers_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_numbers", add);
  ROS_INFO("Ready to add number.");
  ros::spin();

  return 0;
}