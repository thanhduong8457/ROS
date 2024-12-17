#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "my_delta_robot/msg/linear_speed_xyz.hpp"
#include "my_delta_robot/msg/posicionxyz.hpp"
#include "my_delta_robot/msg/vmax_amax.hpp"

using namespace std;

typedef struct point {
    double position_x;
    double position_y;
    double position_z;
    int gripper;
} point_t;

enum {
    circle = 0,
    square,
    triangle
};

vector<point_t *> my_point;

double vmax, amax;

double x_current, y_current, z_current, z_start, z_end;
double xx, yy, zz;
double x_circle, y_circle, z_circle;
double x_square, y_square, z_square;
double x_triangle, y_triangle, z_triangle;

std::mutex mProcessingMutex;
std::condition_variable mConditionProcess;
std::atomic<bool> mSignalScheduleSwitch(false);

void add_point(double x, double y, double z, int gripper);
void Status_Delta_Callback(const std_msgs::msg::String::SharedPtr msg);
void node_a_callback(const delta_robot::msg::Posicionxyz::SharedPtr msg);
void set_vmax_amax_callback(const delta_robot::msg::VmaxAmax::SharedPtr msg);
void set_current_point_callback(const delta_robot::msg::Posicionxyz::SharedPtr msg);

/// @brief 
/// @param argc 
/// @param argv 
/// @return 
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("node_b");

    auto receive_node_a = node->create_subscription<delta_robot::msg::Posicionxyz>("send_to_node_b", 10, node_a_callback);
    auto set_vmax_amax = node->create_subscription<delta_robot::msg::VmaxAmax>("set_vmax_amax", 10, set_vmax_amax_callback);
    auto set_current_point = node->create_subscription<delta_robot::msg::Posicionxyz>("set_current_point", 10, set_current_point_callback);
    auto sub_status_delta = node->create_subscription<std_msgs::msg::String>("status_delta", 10, Status_Delta_Callback);

    auto chatter_pub = node->create_publisher<delta_robot::msg::LinearSpeedXYZ>("input_ls_final", 10);
    auto status_to_node_a = node->create_publisher<std_msgs::msg::String>("status_to_node_a", 10);

    rclcpp::Rate loop_rate(1);

    delta_robot::msg::LinearSpeedXYZ linear_speed_xyz;
    std_msgs::msg::String msg;

    // Setting default values
    x_current = 0.0;
    y_current = 0.0;
    z_current = -375.0;

    x_circle = -100.0;
    y_circle = -100.0;
    z_circle = -453.0;

    x_square = 0.0;
    y_square = -100.0;
    z_square = -453.0;

    x_triangle = 100.0;
    y_triangle = -100.0;
    z_triangle = -453.0;

    z_start = 20;
    z_end = 10;

    vmax = 15000.0;
    amax = 130000.0;

    while (rclcpp::ok()) {
        std::unique_lock<std::mutex> uLock(mProcessingMutex);
        mConditionProcess.wait(uLock, [&] { return mSignalScheduleSwitch.load(); });

        linear_speed_xyz.xo = my_point[0]->position_x;
        linear_speed_xyz.yo = my_point[0]->position_y;
        linear_speed_xyz.zo = my_point[0]->position_z;

        linear_speed_xyz.xf = my_point[1]->position_x;
        linear_speed_xyz.yf = my_point[1]->position_y;
        linear_speed_xyz.zf = my_point[1]->position_z;

        linear_speed_xyz.vmax = vmax;
        linear_speed_xyz.amax = amax;

        linear_speed_xyz.gripper = my_point[0]->gripper;

        loop_rate.sleep();
        chatter_pub->publish(linear_speed_xyz);

        cout << "move from: " 
            << my_point[0]->position_x << " " 
            << my_point[0]->position_y << " " 
            << my_point[0]->position_z
            << " to: " 
            << my_point[1]->position_x << " "
            << my_point[1]->position_y << " " 
            << my_point[1]->position_z << endl;

        delete my_point[0];
        my_point.erase(my_point.begin());

        msg.data = "Point [" + to_string(xx) + " " + to_string(yy) + " " + to_string(zz) + "] is finished";
        status_to_node_a->publish(msg);

        mSignalScheduleSwitch = false;
        rclcpp::spin_some(node);
    }

    return 0;
}

/// @brief common function for add point
/// @param x 
/// @param y 
/// @param z 
/// @param gripper 
void add_point(double x, double y, double z, int gripper) {
    point_t *data = new point_t;

    data->position_x = x;
    data->position_y = y;
    data->position_z = z;
    data->gripper = gripper;

    my_point.push_back(data);
}

/// @brief node a is done process -> then process for the next point
/// @param msg 
void Status_Delta_Callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "status from main_node: [%s]\n", msg->data.c_str());

    if (my_point.size() != 0) {
        x_current = my_point[0]->position_x;
        y_current = my_point[0]->position_y;
        z_current = my_point[0]->position_z;
        my_point.clear();

        std::lock_guard<std::mutex> lg(mProcessingMutex);
        mSignalScheduleSwitch = true;
        mConditionProcess.notify_one();
    }
}

/// @brief receive point to grip from node a thought "send_to_node_b", -> create the action for process by add 3 sub moveverment
/// @param msg 
void node_a_callback(const delta_robot::msg::Posicionxyz::SharedPtr msg) {
    xx = msg->x0;
    yy = msg->y0;
    zz = msg->z0;
    int type = msg->type;

    cout << "Processing point x: " << xx << " y: " << yy << " z: " << zz << endl;

    add_point(x_current, y_current, z_current, 0);

    add_point(xx, yy, zz, 0);
    add_point(xx, yy, zz - z_start, 1);
    add_point(xx, yy, zz, 1);

    switch (type) {
    case circle:
        add_point(x_circle, y_circle, z_circle, 1);
        add_point(x_circle, y_circle, z_circle - z_end, 0);
        add_point(x_circle, y_circle, z_circle, 0);
        break;

    case square:
        add_point(x_square, y_square, z_square, 1);
        add_point(x_square, y_square, z_square - z_end, 0);
        add_point(x_square, y_square, z_square, 0);
        break;

    case triangle:
        add_point(x_triangle, y_triangle, z_triangle, 1);
        add_point(x_triangle, y_triangle, z_triangle - z_end, 0);
        add_point(x_triangle, y_triangle, z_triangle, 0);
        break;

    default:
        break;
    }

    std::lock_guard<std::mutex> lg(mProcessingMutex);
    mSignalScheduleSwitch = true;
    mConditionProcess.notify_one();
}

/// @brief 
/// @param msg 
void set_vmax_amax_callback(const delta_robot::msg::VmaxAmax::SharedPtr msg) {
    vmax = msg->vmax;
    amax = msg->amax;
    cout << "set vmax = " << vmax << ", and set amax = " << amax << endl;
}

/// @brief 
/// @param msg 
void set_current_point_callback(const delta_robot::msg::Posicionxyz::SharedPtr msg) {
    int temp = msg->type;

    switch (temp) {
    case (-1):
        x_current = msg->x0;
        y_current = msg->y0;
        z_current = msg->z0;

        if (z_current > -375 || z_current < -480) {
            z_current = -375.0;
            cout << "[ERROR] Invalid point, current point now set to x: " << x_current << " y: " << y_current << " z: " << z_current << " y: " << y_current << " z: " << z_current << endl;
        }
        break;

    case (0):
        x_circle = msg->x0;
        y_circle = msg->y0;
        z_circle = msg->z0;
        cout << "Position to put CIRCLE is set to x: " << x_circle << " y: " << y_circle << " z: " << z_circle << endl;
        break;

    case (1):
        x_square = msg->x0;
        y_square = msg->y0;
        z_square = msg->z0;
        cout << "Position to put SQUARE is set to x: " << x_square << " y: " << y_square << " z: " << z_square << endl;
        break;

    case (2):
        x_triangle = msg->x0;
        y_triangle = msg->y0;
        z_triangle = msg->z0;
        cout << "Position to put TRIANGLE is set to x: " << x_triangle << " y: " << y_triangle << " z: " << z_triangle << endl;
        break;

    case (3):
        z_start = msg->x0;
        cout << "The distance to go down and grip when START is set to: " << z_start << endl;
        break;

    case (4):
        z_end = msg->x0;
        cout << "The distance to go down and grip when END is set to: " << z_end << endl;
        break;

    case (5):
        z_start = msg->x0;
        z_end = msg->y0;
        cout << "The distance to go down and grip when START is set to: " << z_start << endl;
        cout << "The distance to go down and grip when END is set to: " << z_end << endl;
        break;

    default:
        break;
    }
}