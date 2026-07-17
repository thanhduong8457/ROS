#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <memory>
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

enum class LegacyTargetType : int {
  kCircle = 0,
  kRectangle = 1,
  kTriangle = 2,
};

enum class DrawCommandType : int {
  kCurrentPoint = -1,
  kPathPointA = 0,
  kPathPointB = 1,
  kPathPointC = 2,
  kDrawOffset = 3,
  kLegacyTargetOffset = 4,
  kBothOffsets = 5,
  kDrawRectangle = 6,
  kDrawTriangle = 7,
  kDrawCircle = 8,
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
  return status.rfind("DONE", 0) == 0 ||
         (status.size() >= 4 &&
          status.compare(status.size() - 4, 4, "DONE") == 0);
}

} // namespace

class DrawNode : public rclcpp::Node {
public:
  DrawNode()
      : Node("draw_node"),
        path_points_{Point{0.0, 10.0, -453.0}, Point{-10.0, 0.0, -453.0},
                     Point{0.0, -10.0, -453.0}, Point{10.0, 0.0, -453.0}} {
    RCLCPP_INFO(get_logger(), "draw_node started");

    legacy_point_subscription_ =
        create_subscription<my_delta_robot::msg::Posicionxyz>(
            "send_to_node_b", 10,
            std::bind(&DrawNode::onLegacyPointCommand, this,
                      std::placeholders::_1));
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
    legacy_status_publisher_ =
        create_publisher<std_msgs::msg::String>("status_to_node_a", 10);
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
      finishLegacyCommand(true, "Legacy point sequence completed");
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
      motion_in_flight_ = MotionKind::kNone;
      circle_pending_ = false;
      waypoint_queue_.clear();
      finishLegacyCommand(false, msg->data);
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

  void
  onLegacyPointCommand(const my_delta_robot::msg::Posicionxyz::SharedPtr msg) {
    if (!commandCanStart("Legacy point command")) {
      return;
    }

    const Point requested_point(msg->x0, msg->y0, msg->z0);
    if (!delta_robot_validation::isFinitePoint(requested_point)) {
      RCLCPP_ERROR(get_logger(),
                   "Legacy point command contains non-finite coordinates");
      return;
    }

    const int raw_type = static_cast<int>(msg->type);
    if (raw_type < static_cast<int>(LegacyTargetType::kCircle) ||
        raw_type > static_cast<int>(LegacyTargetType::kTriangle)) {
      RCLCPP_ERROR(get_logger(), "Unknown legacy target type: %d", raw_type);
      return;
    }

    last_legacy_command_ = requested_point;
    legacy_command_active_ = true;
    enqueuePoint(requested_point);
    enqueuePoint({requested_point.x, requested_point.y,
                  requested_point.z - draw_offset_mm_});
    enqueuePoint(requested_point);

    Point target;
    switch (static_cast<LegacyTargetType>(raw_type)) {
    case LegacyTargetType::kCircle:
      target = circle_target_;
      break;
    case LegacyTargetType::kRectangle:
      target = rectangle_target_;
      break;
    case LegacyTargetType::kTriangle:
      target = triangle_target_;
      break;
    }
    enqueuePoint(target);
    enqueuePoint({target.x, target.y, target.z - legacy_target_offset_mm_});
    enqueuePoint(target);
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
    case DrawCommandType::kLegacyTargetOffset:
      setOffset(legacy_target_offset_mm_, msg->x0, "Legacy target Z offset");
      return;
    case DrawCommandType::kBothOffsets:
      if (msg->x0 < 0.0 || msg->y0 < 0.0) {
        RCLCPP_ERROR(get_logger(), "Z offsets must be non-negative");
        return;
      }
      draw_offset_mm_ = msg->x0;
      legacy_target_offset_mm_ = msg->y0;
      RCLCPP_INFO(get_logger(),
                  "Z offsets updated: draw=%.2f legacy_target=%.2f",
                  draw_offset_mm_, legacy_target_offset_mm_);
      return;
    case DrawCommandType::kDrawRectangle:
      replaceQueue(delta_drawing::rectanglePath(path_points_, draw_offset_mm_,
                                                current_point_));
      RCLCPP_INFO(get_logger(), "Rectangle path queued");
      break;
    case DrawCommandType::kDrawTriangle:
      replaceQueue(delta_drawing::trianglePath(path_points_, draw_offset_mm_,
                                               current_point_));
      RCLCPP_INFO(get_logger(), "Triangle path queued");
      break;
    case DrawCommandType::kDrawCircle: {
      const double radius_mm =
          std::max(kDefaultCircleRadiusMm, std::abs(legacy_target_offset_mm_));
      circle_center_mm_ = {
          circle_target_.x,
          circle_target_.y,
          circle_target_.z - draw_offset_mm_,
      };
      circle_radius_mm_ = radius_mm;
      circle_pending_ = true;
      waypoint_queue_.clear();
      enqueuePoint({circle_center_mm_.x + circle_radius_mm_,
                    circle_center_mm_.y, circle_center_mm_.z});
      enqueuePoint(current_point_);
      RCLCPP_INFO(get_logger(),
                  "Continuous circle queued: radius=%.2f mm at 1 kHz",
                  radius_mm);
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
    current_point_ = requested;
    if (current_point_.z > kWorkspaceMaxZMm ||
        current_point_.z < kWorkspaceMinZMm) {
      current_point_.z = kWorkspaceMaxZMm;
      RCLCPP_WARN(get_logger(),
                  "Invalid Z; current point clamped to (%.2f, %.2f, %.2f)",
                  current_point_.x, current_point_.y, current_point_.z);
      return;
    }
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

  void finishLegacyCommand(bool success, const std::string &detail) {
    if (!legacy_command_active_) {
      return;
    }

    std_msgs::msg::String status;
    status.data = success ? "Point [" + std::to_string(last_legacy_command_.x) +
                                " " + std::to_string(last_legacy_command_.y) +
                                " " + std::to_string(last_legacy_command_.z) +
                                "] is finished"
                          : "Point sequence failed: " + detail;
    legacy_status_publisher_->publish(status);
    legacy_command_active_ = false;
  }

  double draw_offset_mm_{20.0};
  double legacy_target_offset_mm_{10.0};
  MotionKind motion_in_flight_{MotionKind::kNone};
  bool legacy_command_active_{false};
  bool circle_pending_{false};
  double circle_radius_mm_{kDefaultCircleRadiusMm};

  std::deque<Point> waypoint_queue_;
  Point current_point_{0.0, 0.0, -375.0};
  std::array<Point, 4> path_points_;
  Point circle_target_{-100.0, -100.0, -453.0};
  Point circle_center_mm_;
  Point rectangle_target_{0.0, -100.0, -453.0};
  Point triangle_target_{100.0, -100.0, -453.0};
  Point last_legacy_command_;

  rclcpp::Subscription<my_delta_robot::msg::Posicionxyz>::SharedPtr
      legacy_point_subscription_;
  rclcpp::Subscription<my_delta_robot::msg::Posicionxyz>::SharedPtr
      draw_command_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
  rclcpp::Publisher<my_delta_robot::msg::LinearSpeedXYZ>::SharedPtr
      segment_publisher_;
  rclcpp::Publisher<my_delta_robot::msg::CircleXYZ>::SharedPtr
      circle_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr legacy_status_publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DrawNode>());
  rclcpp::shutdown();
  return 0;
}
