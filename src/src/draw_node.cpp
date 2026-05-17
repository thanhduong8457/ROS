#include <memory>
#include <string>
#include <vector>

#include "common.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "my_delta_robot/msg/linear_speed_xyz.hpp"
#include "my_delta_robot/msg/posicionxyz.hpp"

namespace {

enum ShapeType : int {
    kCircle = 0,
    kSquare,
    kTriangle,
};

enum SetPointType : int {
    kCurrentPoint = -1,
    kCirclePosition = 0,
    kSquarePosition,
    kTrianglePosition,
    kZStart,
    kZEnd,
    kZStartEnd,
    kDrawSquare,
    kDrawTriangle,
};

constexpr double kDefaultZMin = -480.0;
constexpr double kDefaultZMax = -375.0;

}  // namespace

class DrawNode : public rclcpp::Node {
public:
    DrawNode()
    : Node("draw_node"),
      notify_node_a_(false),
      z_start_(20.0),
      z_end_(10.0),
      waypoint_queue_(),
      current_point_(),
      square_corners_{},
      circle_point_(),
      square_point_(),
      triangle_point_(),
      last_command_{0.0, 0.0, 0.0} {
        RCLCPP_INFO(get_logger(), "draw_node started");

        sub_send_to_draw_ = create_subscription<my_delta_robot::msg::Posicionxyz>(
            "send_to_node_b", 10,
            std::bind(&DrawNode::onExternalPoint, this, std::placeholders::_1));
        sub_set_current_point_ = create_subscription<my_delta_robot::msg::Posicionxyz>(
            "set_current_point", 10,
            std::bind(&DrawNode::onSetCurrentPoint, this, std::placeholders::_1));
        sub_status_delta_ = create_subscription<std_msgs::msg::String>(
            "status_delta", 10,
            std::bind(&DrawNode::onTrajectoryDone, this, std::placeholders::_1));

        pub_linear_speed_ = create_publisher<my_delta_robot::msg::LinearSpeedXYZ>(
            "input_ls_final", 10);
        pub_status_to_node_a_ = create_publisher<std_msgs::msg::String>(
            "status_to_node_a", 10);

        current_point_.x = 0.0;
        current_point_.y = 0.0;
        current_point_.z = -375.0;

        circle_point_.x = -100.0;
        circle_point_.y = -100.0;
        circle_point_.z = -453.0;

        square_point_.x = 0.0;
        square_point_.y = -100.0;
        square_point_.z = -453.0;

        triangle_point_.x = 100.0;
        triangle_point_.y = -100.0;
        triangle_point_.z = -453.0;

        square_corners_[0] = {0.0, z_end_, -453.0};
        square_corners_[1] = {-z_end_, 0.0, -453.0};
        square_corners_[2] = {0.0, -z_end_, -453.0};
        square_corners_[3] = {z_end_, 0.0, -453.0};
    }

private:
    void enqueuePoint(double x, double y, double z) {
        waypoint_queue_.emplace_back(x, y, z);
        RCLCPP_DEBUG(get_logger(), "Waypoint queued: (%.2f, %.2f, %.2f)", x, y, z);
    }

    void publishNextSegment() {
        if (!rclcpp::ok() || waypoint_queue_.empty()) {
            return;
        }

        while (!waypoint_queue_.empty() && current_point_ == waypoint_queue_.front()) {
            RCLCPP_INFO(get_logger(), "Skipping duplicate waypoint");
            waypoint_queue_.erase(waypoint_queue_.begin());
        }
        if (waypoint_queue_.empty()) {
            return;
        }

        const Point & target = waypoint_queue_.front();
        my_delta_robot::msg::LinearSpeedXYZ cmd;
        cmd.xo = current_point_.x;
        cmd.yo = current_point_.y;
        cmd.zo = current_point_.z;
        cmd.xf = target.x;
        cmd.yf = target.y;
        cmd.zf = target.z;
        cmd.gripper = 0;
        pub_linear_speed_->publish(cmd);

        RCLCPP_INFO(
            get_logger(),
            "Move from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)",
            current_point_.x, current_point_.y, current_point_.z,
            target.x, target.y, target.z);

        if (notify_node_a_) {
            std_msgs::msg::String msg;
            msg.data = "Point [" + std::to_string(last_command_.x) + " "
                + std::to_string(last_command_.y) + " "
                + std::to_string(last_command_.z) + "] is finished";
            pub_status_to_node_a_->publish(msg);
            notify_node_a_ = false;
        }
    }

    void onTrajectoryDone(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "status from main_node: %s", msg->data.c_str());

        if (!waypoint_queue_.empty()) {
            current_point_ = waypoint_queue_.front();
            waypoint_queue_.erase(waypoint_queue_.begin());
        }

        if (!waypoint_queue_.empty()) {
            notify_node_a_ = true;
            publishNextSegment();
        }
    }

    void onExternalPoint(const my_delta_robot::msg::Posicionxyz::SharedPtr msg) {
        last_command_.x = msg->x0;
        last_command_.y = msg->y0;
        last_command_.z = msg->z0;
        const int type = static_cast<int>(msg->type);

        RCLCPP_INFO(
            get_logger(), "External point (%.2f, %.2f, %.2f) type=%d",
            last_command_.x, last_command_.y, last_command_.z, type);

        enqueuePoint(current_point_.x, current_point_.y, current_point_.z);
        enqueuePoint(last_command_.x, last_command_.y, last_command_.z);
        enqueuePoint(last_command_.x, last_command_.y, last_command_.z - z_start_);
        enqueuePoint(last_command_.x, last_command_.y, last_command_.z);

        switch (type) {
            case kCircle:
                enqueuePoint(circle_point_.x, circle_point_.y, circle_point_.z);
                enqueuePoint(circle_point_.x, circle_point_.y, circle_point_.z - z_end_);
                enqueuePoint(circle_point_.x, circle_point_.y, circle_point_.z);
                break;
            case kSquare:
                enqueuePoint(square_point_.x, square_point_.y, square_point_.z);
                enqueuePoint(square_point_.x, square_point_.y, square_point_.z - z_end_);
                enqueuePoint(square_point_.x, square_point_.y, square_point_.z);
                break;
            case kTriangle:
                enqueuePoint(triangle_point_.x, square_point_.y, square_point_.z);
                enqueuePoint(triangle_point_.x, square_point_.y, square_point_.z - z_end_);
                enqueuePoint(triangle_point_.x, square_point_.y, square_point_.z);
                break;
            default:
                break;
        }

        publishNextSegment();
    }

    void onSetCurrentPoint(const my_delta_robot::msg::Posicionxyz::SharedPtr msg) {
        const int type = static_cast<int>(msg->type);
        bool should_publish = true;
        waypoint_queue_.clear();

        switch (type) {
            case kCurrentPoint:
                current_point_.x = msg->x0;
                current_point_.y = msg->y0;
                current_point_.z = msg->z0;
                if (current_point_.z > kDefaultZMax || current_point_.z < kDefaultZMin) {
                    current_point_.z = kDefaultZMax;
                    RCLCPP_WARN(
                        get_logger(),
                        "Invalid Z; current point clamped to (%.2f, %.2f, %.2f)",
                        current_point_.x, current_point_.y, current_point_.z);
                } else {
                    RCLCPP_INFO(
                        get_logger(),
                        "Current point set to (%.2f, %.2f, %.2f)",
                        current_point_.x, current_point_.y, current_point_.z);
                }
                should_publish = false;
                break;

            case kCirclePosition:
                square_corners_[0].x = msg->x0;
                square_corners_[0].y = msg->y0;
                square_corners_[0].z = msg->z0;
                RCLCPP_INFO(get_logger(), "Circle corner A updated");
                break;

            case kSquarePosition:
                square_corners_[1].x = msg->x0;
                square_corners_[1].y = msg->y0;
                square_corners_[1].z = msg->z0;
                RCLCPP_INFO(get_logger(), "Square corner B updated");
                break;

            case kTrianglePosition:
                square_corners_[2].x = msg->x0;
                square_corners_[2].y = msg->y0;
                square_corners_[2].z = msg->z0;
                RCLCPP_INFO(get_logger(), "Triangle corner C updated");
                break;

            case kZStart:
                z_start_ = msg->x0;
                RCLCPP_INFO(get_logger(), "Z start offset set to %.2f", z_start_);
                break;

            case kZEnd:
                z_end_ = msg->x0;
                RCLCPP_INFO(get_logger(), "Z end offset set to %.2f", z_end_);
                break;

            case kZStartEnd:
                z_start_ = msg->x0;
                z_end_ = msg->y0;
                RCLCPP_INFO(
                    get_logger(), "Z offsets set: start=%.2f end=%.2f", z_start_, z_end_);
                break;

            case kDrawSquare:
                queueSquarePath();
                break;

            case kDrawTriangle:
                queueTrianglePath();
                break;

            default:
                RCLCPP_WARN(get_logger(), "Unknown set_current_point type: %d", type);
                should_publish = false;
                break;
        }

        if (should_publish) {
            publishNextSegment();
        }
    }

    void queueSquarePath() {
        RCLCPP_INFO(get_logger(), "Queueing square draw path");
        enqueuePoint(square_corners_[0].x, square_corners_[0].y, square_corners_[0].z - z_start_);
        enqueuePoint(square_corners_[1].x, square_corners_[1].y, square_corners_[1].z - z_start_);
        enqueuePoint(square_corners_[2].x, square_corners_[2].y, square_corners_[2].z - z_start_);
        enqueuePoint(square_corners_[3].x, square_corners_[3].y, square_corners_[3].z - z_start_);
        enqueuePoint(square_corners_[0].x, square_corners_[0].y, square_corners_[0].z - z_start_);
        enqueuePoint(current_point_.x, current_point_.y, current_point_.z);
    }

    void queueTrianglePath() {
        RCLCPP_INFO(get_logger(), "Queueing triangle draw path");
        enqueuePoint(square_corners_[0].x, square_corners_[0].y, square_corners_[0].z - z_start_);
        enqueuePoint(square_corners_[1].x, square_corners_[1].y, square_corners_[1].z - z_start_);
        enqueuePoint(square_corners_[2].x, square_corners_[2].y, square_corners_[2].z - z_start_);
        enqueuePoint(square_corners_[0].x, square_corners_[0].y, square_corners_[0].z - z_start_);
        enqueuePoint(current_point_.x, current_point_.y, current_point_.z);
    }

    bool notify_node_a_;
    double z_start_;
    double z_end_;

    std::vector<Point> waypoint_queue_;
    Point current_point_;
    Point square_corners_[4];
    Point circle_point_;
    Point square_point_;
    Point triangle_point_;
    Point last_command_;

    rclcpp::Subscription<my_delta_robot::msg::Posicionxyz>::SharedPtr sub_send_to_draw_;
    rclcpp::Subscription<my_delta_robot::msg::Posicionxyz>::SharedPtr sub_set_current_point_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_delta_;

    rclcpp::Publisher<my_delta_robot::msg::LinearSpeedXYZ>::SharedPtr pub_linear_speed_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_to_node_a_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrawNode>());
    rclcpp::shutdown();
    return 0;
}
