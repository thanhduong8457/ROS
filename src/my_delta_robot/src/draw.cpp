#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>

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

enum {
    CURRENT_POINT = -1,
    CIRCLE_POSITION = 0,
    SQUARE_POSITION,
    TRIANGLE,
    Z_START,
    Z_END,
    Z_START_END,
    DRAW_SQUARE,
    DRAW_TRIANGLE
};

vector<point_t *> my_point;
bool status(false);
bool is_done(true);

double vmax(10000.0), amax(100000.0);

double x_current(0.0), y_current(0.0), z_current(-375.0), z_start(25), z_end(50);

double x_A, y_A, z_A;
double x_B, y_B, z_B;
double x_C, y_C, z_C;
double x_D, y_D, z_D;

void add_point(double x, double y, double z, int gripper);
void Status_Delta_Callback(const std_msgs::msg::String::SharedPtr msg);
void set_vmax_amax_callback(const my_delta_robot::msg::VmaxAmax::SharedPtr msg);
void set_current_point_callback(const my_delta_robot::msg::Posicionxyz::SharedPtr msg);
void draw_new_square();
void draw_new_triangle();

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("node_b");

    auto set_vmax_amax = node->create_subscription<my_delta_robot::msg::VmaxAmax>(
        "set_vmax_amax", 10, set_vmax_amax_callback);
    auto set_current_point = node->create_subscription<my_delta_robot::msg::Posicionxyz>(
        "set_current_point", 10, set_current_point_callback);
    auto sub_status_delta = node->create_subscription<std_msgs::msg::String>(
        "status_delta", 10, Status_Delta_Callback);

    auto chatter_pub = node->create_publisher<my_delta_robot::msg::LinearSpeedXYZ>("input_ls_final", 10);
    auto status_to_node_a = node->create_publisher<std_msgs::msg::String>("status_to_node_a", 10);

    rclcpp::Rate loop_rate(1);

    my_delta_robot::msg::LinearSpeedXYZ linear_speed_xyz;
    std_msgs::msg::String msg;

    // Setting default values
    x_A = 0.0;
    y_A = z_end;
    z_A = -453.0;

    x_B = -z_end;
    y_B = 0.0;
    z_B = -453.0;

    x_C = 0.0;
    y_C = -z_end;
    z_C = -453.0;

    x_D = z_end;
    y_D = 0.0;
    z_D = -453.0;

    while (rclcpp::ok()) {
        if (status) {
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

            cout << "move from: " << my_point[0]->position_x << " " << my_point[0]->position_y << " " << my_point[0]->position_z
                 << " to: " << my_point[1]->position_x << " " << my_point[1]->position_y << " " << my_point[1]->position_z << endl;

            delete my_point[0];
            my_point.erase(my_point.begin());

            status = false;
        }

        rclcpp::spin_some(node);
    }

    return 0;
}

void add_point(double x, double y, double z, int gripper) {
    point_t *data = new point_t;

    data->position_x = x;
    data->position_y = y;
    data->position_z = z;
    data->gripper = gripper;

    my_point.push_back(data);
}

void Status_Delta_Callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "status from main_node: [%s]", msg->data.c_str());

    if (my_point.size() == 1) {
        x_current = my_point[0]->position_x;
        y_current = my_point[0]->position_y;
        z_current = my_point[0]->position_z;

        my_point.clear();

        is_done = true;

        cout << endl;
    }

    if (!my_point.empty()) {
        status = true;
    }
}

void draw_new_square() {
    cout << "Draw Square" << endl;
    add_point(x_current, y_current, z_current, 0);
    add_point(x_A, y_A, z_A - z_start, 0);
    add_point(x_B, y_B, z_B - z_start, 0);
    add_point(x_C, y_C, z_C - z_start, 0);
    add_point(x_D, y_D, z_D - z_start, 0);
    add_point(x_A, y_A, z_A - z_start, 0);
    add_point(x_current, y_current, z_current, 0);

    status = true;
}

void draw_new_triangle() {
    cout << "Draw Triangle" << endl;
    add_point(x_current, y_current, z_current, 0);
    add_point(x_A, y_A, z_A - z_start, 0);
    add_point(x_B, y_B, z_B - z_start, 0);
    add_point(x_C, y_C, z_C - z_start, 0);
    add_point(x_A, y_A, z_A - z_start, 0);
    add_point(x_current, y_current, z_current, 0);

    status = true;
}

void set_vmax_amax_callback(const my_delta_robot::msg::VmaxAmax::SharedPtr msg) {
    vmax = msg->vmax;
    amax = msg->amax;
    cout << "set vmax = " << vmax << ", and set amax = " << amax << endl;
}

void set_current_point_callback(const my_delta_robot::msg::Posicionxyz::SharedPtr msg) {
    int temp = msg->type;

    switch (temp) {
    case (CURRENT_POINT):
        x_current = msg->x0;
        y_current = msg->y0;
        z_current = msg->z0;

        if (z_current > -375 || z_current < -480) {
            z_current = -375.0;
            cout << "[ERROR] Invalid point, current point now set to x: " << x_current << " y: " << y_current << " z: " << z_current << endl;
        } else {
            cout << "current point set to point x: " << x_current << " y: " << y_current << " z: " << z_current << endl;
        }
        break;

    case (CIRCLE_POSITION):
        x_A = msg->x0;
        y_A = msg->y0;
        z_A = msg->z0;
        cout << "Position to put CIRCLE is set to x: " << x_A << " y: " << y_A << " z: " << z_A << endl;
        break;

    case (SQUARE_POSITION):
        x_B = msg->x0;
        y_B = msg->y0;
        z_B = msg->z0;
        cout << "Position to put SQUARE is set to x: " << x_B << " y: " << y_B << " z: " << z_B << endl;
        break;

    case (TRIANGLE):
        x_C = msg->x0;
        y_C = msg->y0;
        z_C = msg->z0;
        cout << "Position to put TRIANGLE is set to x: " << x_C << " y: " << y_C << " z: " << z_C << endl;
        break;

    case (Z_START):
        z_start = msg->x0;
        cout << "The distance to go down and grip when START is set to: " << z_start << endl;
        break;

    case (Z_END):
        z_end = msg->x0;
        cout << "The distance to go down and grip when END is set to: " << z_end << endl;

        x_A = 0.0;
        y_A = z_end;
        z_A = -453.0;

        x_B = -z_end;
        y_B = 0.0;
        z_B = -453.0;

        x_C = 0.0;
        y_C = -z_end;
        z_C = -453.0;

        x_D = z_end;
        y_D = 0.0;
        z_D = -453.0;

        break;

    case (Z_START_END):
        z_start = msg->x0;
        z_end = msg->y0;
        cout << "The distance to go down and grip when START is set to: " << z_start << endl;
        cout << "The distance to go down and grip when END is set to: " << z_end << endl;
        break;

    case (DRAW_SQUARE):
        if (is_done == true) {
            draw_new_square();
            is_done = false;
        }
        break;

    case (DRAW_TRIANGLE):
        if (is_done == true) {
            draw_new_triangle();
            is_done = false;
        }
        break;

    default:
        break;
    }
}
