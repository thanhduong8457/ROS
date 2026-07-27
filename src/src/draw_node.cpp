#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "command_validation.hpp"
#include "common.h"
#include "my_delta_robot/msg/circle_xyz.hpp"
#include "my_delta_robot/msg/linear_speed_xyz.hpp"
#include "my_delta_robot/msg/posicionxyz.hpp"
#include "shape_path.hpp"

namespace {

enum class DrawCommandType : std::int64_t {
  kCurrentPoint = my_delta_robot::msg::Posicionxyz::SET_CURRENT_POINT,
  kPathPointA = my_delta_robot::msg::Posicionxyz::SET_PATH_POINT_A,
  kPathPointB = my_delta_robot::msg::Posicionxyz::SET_PATH_POINT_B,
  kPathPointC = my_delta_robot::msg::Posicionxyz::SET_PATH_POINT_C,
  kDrawOffset = my_delta_robot::msg::Posicionxyz::SET_DRAW_OFFSET,
  kDrawRectangle = my_delta_robot::msg::Posicionxyz::DRAW_RECTANGLE,
  kDrawTriangle = my_delta_robot::msg::Posicionxyz::DRAW_TRIANGLE,
  kDrawCircle = my_delta_robot::msg::Posicionxyz::DRAW_CIRCLE,
};

constexpr double kWorkspaceMinZMm = -480.0;
constexpr double kWorkspaceMaxZMm = -375.0;
constexpr double kDefaultCircleRadiusMm = 25.0;

enum class MotionKind {
  kNone,
  kLine,
  kCircle,
};

bool isFailureStatus(const std::string &status) {
  return status.rfind("FAILED:", 0) == 0;
}

bool isSuccessStatus(const std::string &status) {
  return status.rfind("DONE", 0) == 0;
}

} // namespace

class DrawNode : public rclcpp::Node {
public:
  DrawNode()
      : Node("draw_node"),
        path_points_{Point{0.0, 10.0, -453.0}, Point{-10.0, 0.0, -453.0},
                     Point{0.0, -10.0, -453.0}, Point{10.0, 0.0, -453.0}} {
    RCLCPP_INFO(get_logger(), "draw_node started");

    draw_offset_mm_ =
        declare_parameter<double>("draw_offset_mm", draw_offset_mm_);
    circle_radius_mm_ =
        declare_parameter<double>("circle_radius_mm", circle_radius_mm_);
    circle_reference_mm_.x =
        declare_parameter<double>("circle_center_x_mm", circle_reference_mm_.x);
    circle_reference_mm_.y =
        declare_parameter<double>("circle_center_y_mm", circle_reference_mm_.y);
    circle_reference_mm_.z =
        declare_parameter<double>("circle_base_z_mm", circle_reference_mm_.z);
    if (!std::isfinite(draw_offset_mm_) || draw_offset_mm_ < 0.0) {
      throw std::invalid_argument(
          "draw_offset_mm must be finite and non-negative");
    }
    if (!std::isfinite(circle_radius_mm_) || circle_radius_mm_ <= 0.0) {
      throw std::invalid_argument(
          "circle_radius_mm must be finite and positive");
    }
    if (!delta_robot_validation::isFinitePoint(circle_reference_mm_)) {
      throw std::invalid_argument("circle center parameters must be finite");
    }

    draw_command_subscription_ =
        create_subscription<my_delta_robot::msg::Posicionxyz>(
            "set_current_point", 10,
            std::bind(&DrawNode::onDrawCommand, this, std::placeholders::_1));
    status_subscription_ = create_subscription<std_msgs::msg::String>(
        "status_delta", 10,
        std::bind(&DrawNode::onTrajectoryStatus, this, std::placeholders::_1));

    segment_publisher_ = create_publisher<my_delta_robot::msg::LinearSpeedXYZ>(
        "input_ls_final", 10);
    circle_publisher_ =
        create_publisher<my_delta_robot::msg::CircleXYZ>("input_circle", 10);
    drawing_status_publisher_ =
        create_publisher<std_msgs::msg::String>("drawing_status", 10);
  }

private:
  bool commandCanStart(const char *command_name) const {
    if (motion_in_flight_ == MotionKind::kNone && waypoint_queue_.empty()) {
      return true;
    }
    RCLCPP_WARN(get_logger(),
                "%s rejected because a drawing command is already active",
                command_name);
    return false;
  }

  void replaceQueue(const std::vector<Point> &path) {
    waypoint_queue_.clear();
    waypoint_queue_.insert(waypoint_queue_.end(), path.begin(), path.end());
  }

  void enqueuePoint(const Point &point) {
    waypoint_queue_.push_back(point);
    RCLCPP_DEBUG(get_logger(), "Waypoint queued: (%.2f, %.2f, %.2f)", point.x,
                 point.y, point.z);
  }

  void publishNextSegment() {
    if (!rclcpp::ok() || motion_in_flight_ != MotionKind::kNone) {
      return;
    }

    while (!waypoint_queue_.empty() &&
           delta_drawing::pointsNear(current_point_, waypoint_queue_.front())) {
      RCLCPP_DEBUG(get_logger(), "Skipping duplicate waypoint");
      waypoint_queue_.pop_front();
    }
    if (waypoint_queue_.empty()) {
      finishDrawing(true, "completed");
      return;
    }

    const Point &target = waypoint_queue_.front();
    my_delta_robot::msg::LinearSpeedXYZ command;
    command.xo = current_point_.x;
    command.yo = current_point_.y;
    command.zo = current_point_.z;
    command.xf = target.x;
    command.yf = target.y;
    command.zf = target.z;
    command.gripper = 0;
    segment_publisher_->publish(command);
    motion_in_flight_ = MotionKind::kLine;

    RCLCPP_INFO(get_logger(),
                "Move from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)",
                current_point_.x, current_point_.y, current_point_.z, target.x,
                target.y, target.z);
  }

  void onTrajectoryStatus(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "status from main_node: %s", msg->data.c_str());

    if (isFailureStatus(msg->data)) {
      if (motion_in_flight_ == MotionKind::kNone) {
        RCLCPP_DEBUG(get_logger(),
                     "Ignoring failure with no drawing motion in flight");
        return;
      }
      motion_in_flight_ = MotionKind::kNone;
      circle_pending_ = false;
      waypoint_queue_.clear();
      finishDrawing(false, msg->data);
      RCLCPP_ERROR(get_logger(), "Drawing sequence stopped: %s",
                   msg->data.c_str());
      return;
    }
    if (!isSuccessStatus(msg->data)) {
      RCLCPP_WARN(get_logger(), "Ignoring unrecognized trajectory status");
      return;
    }
    if (motion_in_flight_ == MotionKind::kNone) {
      RCLCPP_DEBUG(get_logger(),
                   "Ignoring status with no drawing segment in flight");
      return;
    }

    if (motion_in_flight_ == MotionKind::kLine) {
      if (waypoint_queue_.empty()) {
        RCLCPP_ERROR(get_logger(), "Line completed with no queued target");
        motion_in_flight_ = MotionKind::kNone;
        return;
      }
      current_point_ = waypoint_queue_.front();
      waypoint_queue_.pop_front();
      motion_in_flight_ = MotionKind::kNone;
      if (circle_pending_) {
        publishCircle();
        return;
      }
    } else {
      motion_in_flight_ = MotionKind::kNone;
      circle_pending_ = false;
    }
    publishNextSegment();
  }

  void onDrawCommand(const my_delta_robot::msg::Posicionxyz::SharedPtr msg) {
    if (!commandCanStart("Draw command")) {
      return;
    }

    const Point value(msg->x0, msg->y0, msg->z0);
    if (!delta_robot_validation::isFinitePoint(value)) {
      RCLCPP_ERROR(get_logger(), "Draw command contains non-finite values");
      return;
    }

    const auto type = static_cast<DrawCommandType>(static_cast<int>(msg->type));
    switch (type) {
    case DrawCommandType::kCurrentPoint:
      setCurrentPoint(value);
      return;
    case DrawCommandType::kPathPointA:
    case DrawCommandType::kPathPointB:
    case DrawCommandType::kPathPointC:
      setPathPoint(type, value);
      return;
    case DrawCommandType::kDrawOffset:
      setOffset(draw_offset_mm_, msg->x0, "Draw Z offset");
      return;
    case DrawCommandType::kDrawRectangle:
      startDrawing("rectangle");
      replaceQueue(delta_drawing::rectanglePath(path_points_, draw_offset_mm_,
                                                current_point_));
      RCLCPP_INFO(get_logger(), "Rectangle path queued");
      break;
    case DrawCommandType::kDrawTriangle:
      startDrawing("triangle");
      replaceQueue(delta_drawing::trianglePath(path_points_, draw_offset_mm_,
                                               current_point_));
      RCLCPP_INFO(get_logger(), "Triangle path queued");
      break;
    case DrawCommandType::kDrawCircle: {
      startDrawing("circle");
      circle_center_mm_ = {
          circle_reference_mm_.x,
          circle_reference_mm_.y,
          circle_reference_mm_.z - draw_offset_mm_,
      };
      circle_pending_ = true;
      waypoint_queue_.clear();
      enqueuePoint({circle_center_mm_.x + circle_radius_mm_,
                    circle_center_mm_.y, circle_center_mm_.z});
      enqueuePoint(current_point_);
      RCLCPP_INFO(get_logger(),
                  "Continuous circle queued: radius=%.2f mm at 1 kHz",
                  circle_radius_mm_);
      break;
    }
    default:
      RCLCPP_WARN(get_logger(), "Unknown set_current_point type: %lld",
                  static_cast<long long>(msg->type));
      return;
    }

    publishNextSegment();
  }

  void setCurrentPoint(const Point &requested) {
    if (requested.z > kWorkspaceMaxZMm || requested.z < kWorkspaceMinZMm) {
      RCLCPP_ERROR(get_logger(),
                   "Current point rejected: Z %.2f is outside [%.2f, %.2f] mm",
                   requested.z, kWorkspaceMinZMm, kWorkspaceMaxZMm);
      return;
    }
    current_point_ = requested;
    RCLCPP_INFO(get_logger(), "Current point set to (%.2f, %.2f, %.2f)",
                current_point_.x, current_point_.y, current_point_.z);
  }

  void setPathPoint(DrawCommandType type, const Point &point) {
    const auto index = static_cast<std::size_t>(static_cast<int>(type));
    path_points_[index] = point;
    RCLCPP_INFO(get_logger(), "Path point %c updated to (%.2f, %.2f, %.2f)",
                static_cast<char>('A' + index), point.x, point.y, point.z);
  }

  void setOffset(double &destination, double value, const char *label) {
    if (value < 0.0) {
      RCLCPP_ERROR(get_logger(), "%s must be non-negative", label);
      return;
    }
    destination = value;
    RCLCPP_INFO(get_logger(), "%s set to %.2f mm", label, destination);
  }

  void publishCircle() {
    my_delta_robot::msg::CircleXYZ command;
    command.center_x = circle_center_mm_.x;
    command.center_y = circle_center_mm_.y;
    command.center_z = circle_center_mm_.z;
    command.radius = circle_radius_mm_;
    command.clockwise = false;
    circle_publisher_->publish(command);
    motion_in_flight_ = MotionKind::kCircle;
    RCLCPP_INFO(get_logger(),
                "Drawing continuous circle: center=(%.2f, %.2f, %.2f), "
                "radius=%.2f mm",
                circle_center_mm_.x, circle_center_mm_.y, circle_center_mm_.z,
                circle_radius_mm_);
  }

  void startDrawing(const std::string &name) {
    active_drawing_ = name;
    std_msgs::msg::String status;
    status.data = "STARTED: " + name;
    drawing_status_publisher_->publish(status);
  }

  void finishDrawing(bool success, const std::string &detail) {
    if (active_drawing_.empty()) {
      return;
    }
    std_msgs::msg::String status;
    status.data = (success ? "DONE: " : "FAILED: ") + active_drawing_;
    if (!success && !detail.empty()) {
      status.data += " - " + detail;
    }
    drawing_status_publisher_->publish(status);
    active_drawing_.clear();
  }

  double draw_offset_mm_{20.0};
  MotionKind motion_in_flight_{MotionKind::kNone};
  bool circle_pending_{false};
  double circle_radius_mm_{kDefaultCircleRadiusMm};

  std::deque<Point> waypoint_queue_;
  Point current_point_{0.0, 0.0, -375.0};
  std::array<Point, 4> path_points_;
  Point circle_reference_mm_{-100.0, -100.0, -453.0};
  Point circle_center_mm_;
  std::string active_drawing_;

  rclcpp::Subscription<my_delta_robot::msg::Posicionxyz>::SharedPtr
      draw_command_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
  rclcpp::Publisher<my_delta_robot::msg::LinearSpeedXYZ>::SharedPtr
      segment_publisher_;
  rclcpp::Publisher<my_delta_robot::msg::CircleXYZ>::SharedPtr
      circle_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr drawing_status_publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DrawNode>());
  rclcpp::shutdown();
  return 0;
}
