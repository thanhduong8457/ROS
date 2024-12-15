#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <csignal>

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

class Draw : public rclcpp::Node {
public:
    Draw() : Node("draw"), loop_rate(1) {
        RCLCPP_INFO(this->get_logger(), "draw is created");

        receive_node_a = this->create_subscription<my_delta_robot::msg::Posicionxyz>("send_to_node_b", 10, std::bind(&Draw::node_a_callback, this, std::placeholders::_1));
        set_vmax_amax = this->create_subscription<my_delta_robot::msg::VmaxAmax>("set_vmax_amax", 10, std::bind(&Draw::set_vmax_amax_callback, this, std::placeholders::_1));
        set_current_point = this->create_subscription<my_delta_robot::msg::Posicionxyz>("set_current_point", 10, std::bind(&Draw::set_current_point_callback, this, std::placeholders::_1));
        sub_status_delta = this->create_subscription<std_msgs::msg::String>("status_delta", 10, std::bind(&Draw::Status_Delta_Callback, this, std::placeholders::_1));

        chatter_pub = this->create_publisher<my_delta_robot::msg::LinearSpeedXYZ>("input_ls_final", 10);
        status_to_node_a = this->create_publisher<std_msgs::msg::String>("status_to_node_a", 10);

        // Setting default values
        mCurrentPoint.position_x = 0.0;
        mCurrentPoint.position_y = 0.0;
        mCurrentPoint.position_z = -375.0;

        mCirclePoint.position_x = -100.0;
        mCirclePoint.position_y = -100.0;
        mCirclePoint.position_z = -453.0;

        mSquarePoint.position_x = 0.0;
        mSquarePoint.position_y = -100.0;
        mSquarePoint.position_z = -453.0;

        mTrianglePoint.position_x = 100.0;
        mTrianglePoint.position_y = -100.0;
        mTrianglePoint.position_z = -453.0;

        z_start = 20;
        z_end = 10;

        mPointA.position_x = 0.0;
        mPointA.position_y = z_end;
        mPointA.position_z = -453.0;

        mPointB.position_x = -z_end;
        mPointB.position_y = 0.0;
        mPointB.position_z = -453.0;

        mPointC.position_x = 0.0;
        mPointC.position_y = -z_end;
        mPointC.position_z = -453.0;

        mPointD.position_x = z_end;
        mPointD.position_y = 0.0;
        mPointD.position_z = -453.0;

        vmax = 15000.0;
        amax = 130000.0;

        status = false;
        is_send_status_to_node_a = false;

        // Create a thread to handle the main processing loop
        // processing_thread = std::thread(&Draw::MainProcessFunction, this);
    }

    ~Draw() {

    }

private:
    vector<point_t *> my_point;
    bool status;
    bool is_send_status_to_node_a;

    double vmax, amax;

    double z_start, z_end;
    double xx, yy, zz;

    point mCurrentPoint;
    point mPointA;
    point mPointB;
    point mPointC;
    point mPointD;
    point mCirclePoint;
    point mSquarePoint;
    point mTrianglePoint;

    std::mutex mProcessingMutex;
    std::condition_variable mConditionProcess;
    std::atomic<bool> mSignalScheduleSwitch{false};
    std::atomic<bool> running{true};

    rclcpp::Rate loop_rate;
    std::shared_ptr<rclcpp::Subscription<my_delta_robot::msg::Posicionxyz>> receive_node_a;
    std::shared_ptr<rclcpp::Subscription<my_delta_robot::msg::VmaxAmax>> set_vmax_amax;
    std::shared_ptr<rclcpp::Subscription<my_delta_robot::msg::Posicionxyz>> set_current_point;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> sub_status_delta;
    std::shared_ptr<rclcpp::Publisher<my_delta_robot::msg::LinearSpeedXYZ>> chatter_pub;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> status_to_node_a;

    // std::thread processing_thread;

    void MainProcessFunction() {
        my_delta_robot::msg::LinearSpeedXYZ linear_speed_xyz;
        std_msgs::msg::String msg;
        bool is_proccess_ok = false;
        if (rclcpp::ok()) {
            while (false == is_proccess_ok) {
                if ((mCurrentPoint.position_x == my_point[0]->position_x) &&
                    (mCurrentPoint.position_y == my_point[0]->position_y) &&
                    (mCurrentPoint.position_z == my_point[0]->position_z))
                {
                    RCLCPP_INFO(this->get_logger(), "remove duplicate point\n");
                    delete my_point[0];
                    my_point.erase(my_point.begin());
                    if(0 == my_point.size()) {
                        return;
                    }
                }
                else {
                    RCLCPP_INFO(this->get_logger(), "process for next position\n");
                    is_proccess_ok = true;
                }
            }
            
            linear_speed_xyz.xo = mCurrentPoint.position_x;
            linear_speed_xyz.yo = mCurrentPoint.position_y;
            linear_speed_xyz.zo = mCurrentPoint.position_z;
            linear_speed_xyz.xf = my_point[0]->position_x;
            linear_speed_xyz.yf = my_point[0]->position_y;
            linear_speed_xyz.zf = my_point[0]->position_z;
            linear_speed_xyz.vmax = vmax;
            linear_speed_xyz.amax = amax;
            linear_speed_xyz.gripper = my_point[0]->gripper;
            loop_rate.sleep();
            chatter_pub->publish(linear_speed_xyz);

            cout << "move from: " 
                << mCurrentPoint.position_x << " " 
                << mCurrentPoint.position_y << " " 
                << mCurrentPoint.position_z
                << " to: " 
                << my_point[0]->position_x << " "
                << my_point[0]->position_y << " " 
                << my_point[0]->position_z << endl;

            if (is_send_status_to_node_a) {
                msg.data = "Point [" + to_string(xx) + " " + to_string(yy) + " " + to_string(zz) + "] is finished";
                status_to_node_a->publish(msg);
                is_send_status_to_node_a = false;
            }
        }
    }

    /// @brief 
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

        cout << "point added with x=" << x << ", y=" << y << ", z=" << z << ", gripper=" << gripper << endl;
    }

    /// @brief 
    /// @param msg 
    void Status_Delta_Callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "status from main_node: [%s]\n", msg->data.c_str());

        if (my_point.size() != 0) {
            mCurrentPoint.position_x = my_point[0]->position_x;
            mCurrentPoint.position_y = my_point[0]->position_y;
            mCurrentPoint.position_z = my_point[0]->position_z;
            delete my_point[0];
            my_point.erase(my_point.begin());
            is_send_status_to_node_a = true;
            MainProcessFunction();
        }
    }

    /// @brief 
    /// @param msg 
    void node_a_callback(const my_delta_robot::msg::Posicionxyz::SharedPtr msg) {
        xx = msg->x0;
        yy = msg->y0;
        zz = msg->z0;
        int type = msg->type;

        cout << "Processing point x: " << xx << " y: " << yy << " z: " << zz << endl;

        add_point(mCurrentPoint.position_x, mCurrentPoint.position_y, mCurrentPoint.position_z, 0);

        add_point(xx, yy, zz, 0);
        add_point(xx, yy, zz - z_start, 1);
        add_point(xx, yy, zz, 1);

        switch (type) {
        case circle:
            add_point(mCirclePoint.position_x, mCirclePoint.position_y, mCirclePoint.position_z, 1);
            add_point(mCirclePoint.position_x, mCirclePoint.position_y, mCirclePoint.position_z - z_end, 0);
            add_point(mCirclePoint.position_x, mCirclePoint.position_y, mCirclePoint.position_z, 0);
            break;

        case square:
            add_point(mSquarePoint.position_x, mSquarePoint.position_y, mSquarePoint.position_z, 1);
            add_point(mSquarePoint.position_x, mSquarePoint.position_y, mSquarePoint.position_z - z_end, 0);
            add_point(mSquarePoint.position_x, mSquarePoint.position_y, mSquarePoint.position_z, 0);
            break;

        case triangle:
            add_point(mTrianglePoint.position_x, mSquarePoint.position_y, mSquarePoint.position_z, 1);
            add_point(mTrianglePoint.position_x, mSquarePoint.position_y, mSquarePoint.position_z - z_end, 0);
            add_point(mTrianglePoint.position_x, mSquarePoint.position_y, mSquarePoint.position_z, 0);
            break;

        default:
            break;
        }

        MainProcessFunction();
    }

    /// @brief 
    /// @param msg 
    void set_vmax_amax_callback(const my_delta_robot::msg::VmaxAmax::SharedPtr msg) {
        vmax = msg->vmax;
        amax = msg->amax;
        cout << "set vmax = " << vmax << ", and set amax = " << amax << endl;
    }

    /// @brief 
    /// @param msg 
    void set_current_point_callback(const my_delta_robot::msg::Posicionxyz::SharedPtr msg) {
        int temp = msg->type;
        bool zuo_bu_zuo = true;
        my_point.clear(); // clear all data inside

        switch (temp) {
        case (CURRENT_POINT):
            mCurrentPoint.position_x = msg->x0;
            mCurrentPoint.position_y = msg->y0;
            mCurrentPoint.position_z = msg->z0;

            if (mCurrentPoint.position_z > -375 || mCurrentPoint.position_z < -480) {
                mCurrentPoint.position_z = -375.0;
                cout << "[ERROR] Invalid point, current point now set to x: " << mCurrentPoint.position_x << " y: " << mCurrentPoint.position_y << " z: " << mCurrentPoint.position_z << endl;
            } else {
                cout << "current point set to point x: " << mCurrentPoint.position_x << " y: " << mCurrentPoint.position_y << " z: " << mCurrentPoint.position_z << endl;
            }
            zuo_bu_zuo = false;
            break;

        case (CIRCLE_POSITION):
            mPointA.position_x = msg->x0;
            mPointA.position_y = msg->y0;
            mPointA.position_z = msg->z0;
            cout << "Position to put CIRCLE is set to x: " << mPointA.position_x << " y: " << mPointA.position_y << " z: " << mPointA.position_z << endl;
            break;

        case (SQUARE_POSITION):
            mPointB.position_x = msg->x0;
            mPointB.position_y = msg->y0;
            mPointB.position_z = msg->z0;
            cout << "Position to put SQUARE is set to x: " << mPointB.position_x << " y: " << mPointB.position_y << " z: " << mPointB.position_z << endl;
            break;

        case (TRIANGLE):
            mPointC.position_x = msg->x0;
            mPointC.position_y = msg->y0;
            mPointC.position_z = msg->z0;
            cout << "Position to put TRIANGLE is set to x: " << mPointC.position_x << " y: " << mPointC.position_y << " z: " << mPointC.position_z << endl;
            break;

        case (Z_START):
            z_start = msg->x0;
            cout << "The distance to go down and grip when START is set to: " << z_start << endl;
            break;

        case (Z_END):
            z_end = msg->x0;
            cout << "The distance to go down and grip when END is set to: " << z_end << endl;
            break;

        case (Z_START_END):
            z_start = msg->x0;
            z_end = msg->y0;
            cout << "The distance to go down and grip when START is set to: " << z_start << endl;
            cout << "The distance to go down and grip when END is set to: " << z_end << endl;
            break;

        case (DRAW_SQUARE):
            draw_new_square();
            break;

        case (DRAW_TRIANGLE):
            draw_new_triangle();
            break;

        default:
            cout << "mei you gong zuo hui zuo, qing ni you zuo ba!!!" << endl;
            zuo_bu_zuo = false;
            break;
        }
        
        if (true == zuo_bu_zuo) {
            MainProcessFunction();
        }
    }

    /// @brief 
    void draw_new_square() {
        cout << "Draw Square" << endl;
        // add_point(mCurrentPoint.position_x, mCurrentPoint.position_y, mCurrentPoint.position_z, 0);
        add_point(mPointA.position_x, mPointA.position_y, mPointA.position_z - z_start, 0);
        add_point(mPointB.position_x, mPointB.position_y, mPointB.position_z - z_start, 0);
        add_point(mPointC.position_x, mPointC.position_y, mPointC.position_z - z_start, 0);
        add_point(mPointD.position_x, mPointD.position_y, mPointD.position_z - z_start, 0);
        add_point(mPointA.position_x, mPointA.position_y, mPointA.position_z - z_start, 0);
        add_point(mCurrentPoint.position_x, mCurrentPoint.position_y, mCurrentPoint.position_z, 0);

    }

    /// @brief 
    void draw_new_triangle() {
        cout << "Draw Triangle" << endl;
        // add_point(mCurrentPoint.position_x, mCurrentPoint.position_y, mCurrentPoint.position_z, 0);
        add_point(mPointA.position_x, mPointA.position_y, mPointA.position_z - z_start, 0);
        add_point(mPointB.position_x, mPointB.position_y, mPointB.position_z - z_start, 0);
        add_point(mPointC.position_x, mPointC.position_y, mPointC.position_z - z_start, 0);
        add_point(mPointA.position_x, mPointA.position_y, mPointA.position_z - z_start, 0);
        add_point(mCurrentPoint.position_x, mCurrentPoint.position_y, mCurrentPoint.position_z, 0);
    }
};

/// @brief 
/// @param argc 
/// @param argv 
/// @return 
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Draw>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
