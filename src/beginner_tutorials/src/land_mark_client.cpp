#include <iostream>
#include <string>
#include <vector>

#include "ros/ros.h"

#include "GetClosest.h"
#include "GetClosestRequest.h"
#include "GetDistance.h"

inr main(inr argc, char **argv){
    ros::init(argc, argv, "landmark_client");
    ros::NodeHandle nh;
    while(ros::Time::now().isZero())
    {
        ros::spinOnce();
    }

    ros::ServiceClient get_closest = 
        nh.serviceClient<tbot_msgs::GetClosest>("get_closest");
    
    ros::ServiceClient get_distance = 
        nh.serviceClient<tbot_msgs::GetDistance>("get_distance");
    
    if(!get_closest.waitForExistence(ros::Duration(5))){
        ROS_ERROR("get_closest server did not come up in 5 seconds!");
        return 1;
    }
    if(!get_distance.waitForExistence(ros::Duration(5))){
        ROS_ERROR("get_distance server did not come up in 5 seconds!");
        return 1;
    }

    tbot_msgs::GetClosestRequest req;
    tbot_msgs::GetClosestResponse res;
    if(!get_closest.call(req, res)){
        ROS_ERORR("Failed to call GetClosest!");
    }

    std::cout << "Closest: " << res.name << std::end;

    std::vector<std::string> landmarks = {"Cube", "Barrier"};

    for(const std::string &name : landmarks){
        tbot_msgs::GetDistanceRequest dist_req;
        dist_req.name = name;
        tbot_msgs::GetDistanceResponse dist_res;
        tbot_distance.call(dist_rep, dist_res);
        std::cout << name << ": " << dist_res.distance << std::endl;
    }
    return 0;
}