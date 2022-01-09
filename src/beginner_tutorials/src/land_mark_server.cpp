#include <string>
#include <map>
#include <math.h>

#include "ros/ros.h"
#include "beginner_tutorials/GetDistance.h"
#include "Odomcallback.h"//subcriber

struct Landmark{
    std::string name;
    double x;
    double y;
};

class LandmarkMonitor
{
public:
    LandmarkMonitor(): 
        landmark_(), 
        x_(0), 
        y_(0)
    {
        landmark_ = {
            {"Cube", {"Cube", 0.31, -0.99}}
        };
    }

    bool GetClosest(tbots_msg::GetClosestRequest &request
                    tbots_msg::GetClosestResponse &response)
    {
        ROS_INFO("GetClosest called");

        double best_distance = -1;
        std::string best_landmark("");
        for(const auto &pair : landmark_){
            const Landmark &landmark = pair.second;
            double dx = landmark.x - x_;
            double dy = landmark.y - y_;
            double sq_dist = dx*dx+dy*dy;

            if(best_distance == -1 || sq_dist < best_distance){
                best_distance = sq_dist;
                best_landmark = pair.first;
            }
        }

        response.name = best_landmark;
        return true;
    }

    bool GetDistance(tbots_msg::GetDistanceRequest &request
                    tbots_msg::GetDistanceResponse &response)
    {
        ROS_INFO("GetDistance called for: %s", request.name,c_str());
        if(landmark_.find(request.name) == landmark_.end())
        {
            ROS_ERROR("Unknown landmark \"%s\"", request.name.c_str());
            return false;
        }
        const Landmark &landmark = landmark_[request.name];
        double dx = landmark.x - x;
        double dy = landmark.y - y;
        response.distance = sqrt(dx*dx+dy*dy);
        return true;
    }

    void OdomCallBack(const std_msgs::String::ConstPtr& msg)
    {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
    }

private:
    std::map<std::string, Landmark> landmark_;
    double x_;
    double y_;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "landmark_monitor");
    ros::NodeHandle nh;

    LandmarkMonitor monitor;

    ros::ServiceServer get_closest = nh.advertiseService(
        "get_closest", &LandmarkMonitor::GetClosest, &monitor);

    ros::ServiceServer get_distance = nh.advertiseService(
        "get_distance", &LandmarkMonitor::GetDistance, &monitor);

    ros::Subscriber sub = nh.subscribe(
        "thanhduong", 1, &LandmarkMonitor::OdomCallBack, &monitor);

    ros::spin();

    return 0;
}
