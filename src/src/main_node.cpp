#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

#include "command_validation.hpp"
#include "delta_robot.h"
#include "joint_state_config.hpp"
#include "motion_planner.hpp"
#include "my_delta_robot/msg/circle_xyz.hpp"
#include "my_delta_robot/msg/linear_speed_xyz.hpp"
#include "my_delta_robot/msg/vmax_amax.hpp"

namespace {

using SteadyClock = std::chrono::steady_clock;

constexpr auto kMotionTick = std::chrono::milliseconds(1);
constexpr auto kIdleStatePublishPeriod = std::chrono::milliseconds(200);
constexpr double kMotionSamplePeriodSec = 0.001;
constexpr double kDefaultVelocityMmS = 5000.0;
constexpr double kDefaultAccelerationMmS2 = 100.0;
constexpr double kDeclaredStartToleranceM = 1e-4;
constexpr double kCircleStartToleranceM = 1e-6;

static_assert(DeltaRobot::kJointCount == delta_robot_config::kNumJoints);

Point millimetresToMetres(const Point &point_mm) {
  return {point_mm.x * mmtm, point_mm.y * mmtm, point_mm.z * mmtm};
}

Point metresToMillimetres(const Point &point_m) {
  return {point_m.x * mtmm, point_m.y * mtmm, point_m.z * mtmm};
}

} // namespace

class MainNode : public rclcpp::Node {
public:
  MainNode() : Node("main_node") {
    RCLCPP_INFO(get_logger(), "main_node started");

    const double max_velocity_mm_s =
        declare_parameter<double>("max_velocity_mm_s", kDefaultVelocityMmS);
    const double max_acceleration_mm_s2 = declare_parameter<double>(
        "max_acceleration_mm_s2", kDefaultAccelerationMmS2);
    const std::string parameter_error =
        delta_robot_validation::validateMotionLimits(max_velocity_mm_s,
                                                     max_acceleration_mm_s2);
    if (!parameter_error.empty()) {
      throw std::invalid_argument("Invalid motion-limit parameters: " +
                                  parameter_error);
    }
    motion_limits_ = {max_velocity_mm_s * mmtm, max_acceleration_mm_s2 * mmtm};
    RCLCPP_INFO(get_logger(),
                "Motion limits configured: vmax=%.1f mm/s, amax=%.1f mm/s^2",
                max_velocity_mm_s, max_acceleration_mm_s2);

    segment_subscription_ =
        create_subscription<my_delta_robot::msg::LinearSpeedXYZ>(
            "input_ls_final", 10,
            std::bind(&MainNode::onSegmentCommand, this,
                      std::placeholders::_1));
    circle_subscription_ = create_subscription<my_delta_robot::msg::CircleXYZ>(
        "input_circle", 10,
        std::bind(&MainNode::onCircleCommand, this, std::placeholders::_1));
    motion_limits_subscription_ =
        create_subscription<my_delta_robot::msg::VmaxAmax>(
            "set_vmax_amax", 10,
            std::bind(&MainNode::onMotionLimitsCommand, this,
                      std::placeholders::_1));

    joint_states_publisher_ =
        create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    status_publisher_ =
        create_publisher<std_msgs::msg::String>("status_delta", 10);
    profile_publisher_ =
        create_publisher<my_delta_robot::msg::VmaxAmax>("v_a_out", 10);

    joint_state_.header.frame_id = "base_link";
    joint_state_.name = delta_robot_config::kJointNames;
    joint_state_.position.resize(delta_robot_config::kNumJoints, 0.0);
    if (!setJointStateFromTcp(current_tcp_m_)) {
      throw std::runtime_error("Failed to initialize delta robot home pose");
    }

    publishCurrentJointState();
    motion_timer_ = create_wall_timer(
        kMotionTick, std::bind(&MainNode::onMotionTimer, this));
    idle_state_timer_ = create_wall_timer(
        kIdleStatePublishPeriod, std::bind(&MainNode::onIdleStateTimer, this));
  }

private:
  bool calculateJointPositions(const Point &tcp_m,
                               DeltaRobot::JointPositions &positions,
                               std::string &error) const {
    const auto ik = robot_.inverseChecked(tcp_m);
    if (!ik.ok) {
      error = ik.error;
      return false;
    }
    if (!robot_.createJointStatePositions(tcp_m, ik.theta, positions) ||
        !delta_robot_validation::areFinite(positions)) {
      error = "Joint-state mapping produced invalid values";
      return false;
    }
    return true;
  }

  bool setJointStateFromTcp(const Point &tcp_m) {
    DeltaRobot::JointPositions positions{};
    std::string error;
    if (!calculateJointPositions(tcp_m, positions, error)) {
      RCLCPP_ERROR(get_logger(), "Cannot map TCP to joints: %s", error.c_str());
      return false;
    }

    std::copy(positions.begin(), positions.end(),
              joint_state_.position.begin());
    current_tcp_m_ = tcp_m;
    return true;
  }

  void publishCurrentJointState() {
    joint_state_.header.stamp = now();
    joint_states_publisher_->publish(joint_state_);
  }

  void onIdleStateTimer() {
    if (motion_active_) {
      return;
    }
    publishCurrentJointState();
  }

  void
  onSegmentCommand(const my_delta_robot::msg::LinearSpeedXYZ::SharedPtr msg) {
    if (motion_active_) {
      RCLCPP_WARN(get_logger(), "Motion already active; new segment rejected");
      return;
    }

    const Point declared_start_mm(msg->xo, msg->yo, msg->zo);
    const Point target_mm(msg->xf, msg->yf, msg->zf);
    if (!delta_robot_validation::isFinitePoint(target_mm)) {
      publishFailure("Segment target coordinates must be finite");
      return;
    }

    if (!delta_robot_validation::isFinitePoint(declared_start_mm)) {
      RCLCPP_WARN(get_logger(),
                  "Ignoring non-finite declared segment start; main_node owns "
                  "the authoritative TCP state");
    } else {
      const Point declared_start_m = millimetresToMetres(declared_start_mm);
      if (!delta_robot_validation::pointsNear(declared_start_m, current_tcp_m_,
                                              kDeclaredStartToleranceM)) {
        const Point current_mm = metresToMillimetres(current_tcp_m_);
        RCLCPP_WARN(
            get_logger(),
            "Ignoring stale declared segment start (%.3f, %.3f, %.3f) mm; "
            "using current TCP (%.3f, %.3f, %.3f) mm",
            declared_start_mm.x, declared_start_mm.y, declared_start_mm.z,
            current_mm.x, current_mm.y, current_mm.z);
      }
    }

    startTrajectory(target_mm);
  }

  void onCircleCommand(const my_delta_robot::msg::CircleXYZ::SharedPtr msg) {
    if (motion_active_) {
      RCLCPP_WARN(get_logger(), "Motion already active; circle rejected");
      return;
    }

    const Point center_mm(msg->center_x, msg->center_y, msg->center_z);
    if (!delta_robot_validation::isFinitePoint(center_mm) ||
        !std::isfinite(msg->radius) || msg->radius <= 0.0) {
      publishFailure(
          "Circle center and radius must be finite and radius positive");
      return;
    }

    const Point center_m = millimetresToMetres(center_mm);
    const double radius_m = msg->radius * mmtm;
    const Point circle_start_m{center_m.x + radius_m, center_m.y, center_m.z};
    if (!delta_robot_validation::pointsNear(current_tcp_m_, circle_start_m,
                                            kCircleStartToleranceM)) {
      const Point current_mm = metresToMillimetres(current_tcp_m_);
      const Point circle_start_mm = metresToMillimetres(circle_start_m);
      publishFailure("Circle requires TCP at its start point; current=(" +
                     std::to_string(current_mm.x) + ", " +
                     std::to_string(current_mm.y) + ", " +
                     std::to_string(current_mm.z) + ") mm, required=(" +
                     std::to_string(circle_start_mm.x) + ", " +
                     std::to_string(circle_start_mm.y) + ", " +
                     std::to_string(circle_start_mm.z) + ") mm");
      return;
    }

    delta_motion::CartesianTrajectoryGenerator generator;
    auto plan = generator.planCircle(center_m, radius_m, msg->clockwise,
                                     motion_limits_, kMotionSamplePeriodSec);
    startPlan(std::move(plan), "circle center=(" + std::to_string(center_mm.x) +
                                   ", " + std::to_string(center_mm.y) + ", " +
                                   std::to_string(center_mm.z) +
                                   "), radius=" + std::to_string(msg->radius));
  }

  void
  onMotionLimitsCommand(const my_delta_robot::msg::VmaxAmax::SharedPtr msg) {
    const std::string error =
        delta_robot_validation::validateMotionLimits(msg->vmax, msg->amax);
    if (!error.empty()) {
      RCLCPP_ERROR(get_logger(), "Motion limits rejected: %s", error.c_str());
      return;
    }

    motion_limits_ = {msg->vmax * mmtm, msg->amax * mmtm};
    RCLCPP_INFO(get_logger(),
                "Motion limits updated: vmax=%.1f mm/s, amax=%.1f mm/s^2",
                msg->vmax, msg->amax);
  }

  void startTrajectory(const Point &target_mm) {
    const Point start_m = current_tcp_m_;
    const Point start_mm = metresToMillimetres(start_m);
    delta_motion::CartesianTrajectoryGenerator generator;
    auto plan = generator.planLine(start_m, millimetresToMetres(target_mm),
                                   motion_limits_, kMotionSamplePeriodSec);
    startPlan(std::move(plan), "line (" + std::to_string(start_mm.x) + ", " +
                                   std::to_string(start_mm.y) + ", " +
                                   std::to_string(start_mm.z) + ") -> (" +
                                   std::to_string(target_mm.x) + ", " +
                                   std::to_string(target_mm.y) + ", " +
                                   std::to_string(target_mm.z) + ")");
  }

  void startPlan(delta_motion::PlanResult plan, std::string description) {
    if (!plan.ok) {
      publishFailure("Trajectory rejected: " + plan.error);
      return;
    }
    if (plan.samples.empty()) {
      publishFailure("Trajectory rejected: planner returned no samples");
      return;
    }

    for (const auto &sample : plan.samples) {
      DeltaRobot::JointPositions positions{};
      std::string error;
      if (!calculateJointPositions(sample.position_m, positions, error)) {
        publishFailure("Trajectory rejected at t=" +
                       std::to_string(sample.time_s) + " s: " + error);
        return;
      }
    }

    active_motion_description_ = std::move(description);
    active_samples_ = std::move(plan.samples);
    last_published_sample_index_ = 0;
    motion_started_at_ = SteadyClock::now();
    motion_active_ = true;
    if (!publishSample(active_samples_.front())) {
      publishFailure("Unable to publish first trajectory sample");
      stopMotion();
      return;
    }

    RCLCPP_INFO(get_logger(), "Started %s: %zu samples, %.4f s",
                active_motion_description_.c_str(), active_samples_.size(),
                active_samples_.back().time_s);
  }

  void onMotionTimer() {
    if (!motion_active_ || active_samples_.empty()) {
      return;
    }

    const double elapsed_seconds =
        std::chrono::duration<double>(SteadyClock::now() - motion_started_at_)
            .count();
    const auto first_future_sample = std::upper_bound(
        active_samples_.begin(), active_samples_.end(), elapsed_seconds,
        [](double elapsed, const delta_motion::TrajectorySample &sample) {
          return elapsed < sample.time_s;
        });
    const auto selected = first_future_sample == active_samples_.begin()
                              ? active_samples_.begin()
                              : std::prev(first_future_sample);
    const auto selected_index = static_cast<std::size_t>(
        std::distance(active_samples_.begin(), selected));

    if (selected_index == last_published_sample_index_) {
      return;
    }

    if (!publishSample(*selected)) {
      publishFailure("Joint mapping failed during motion");
      stopMotion();
      return;
    }
    last_published_sample_index_ = selected_index;

    if (selected_index + 1U == active_samples_.size()) {
      publishSuccess();
      stopMotion();
    }
  }

  bool publishSample(const delta_motion::TrajectorySample &sample) {
    DeltaRobot::JointPositions joint_positions{};
    std::string error;
    if (!calculateJointPositions(sample.position_m, joint_positions, error)) {
      RCLCPP_ERROR(get_logger(), "Joint mapping failed during motion: %s",
                   error.c_str());
      return false;
    }

    std::copy(joint_positions.begin(), joint_positions.end(),
              joint_state_.position.begin());
    current_tcp_m_ = sample.position_m;
    joint_state_.header.stamp = now();
    joint_states_publisher_->publish(joint_state_);

    my_delta_robot::msg::VmaxAmax profile;
    profile.vmax = sample.path_velocity_mps * mtmm;
    profile.amax = sample.path_acceleration_mps2 * mtmm;
    profile_publisher_->publish(profile);
    return true;
  }

  void publishSuccess() {
    std_msgs::msg::String status;
    status.data = "DONE " + active_motion_description_;
    status_publisher_->publish(status);
  }

  void publishFailure(const std::string &reason) {
    RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
    std_msgs::msg::String status;
    status.data = "FAILED: " + reason;
    status_publisher_->publish(status);
  }

  void stopMotion() {
    motion_active_ = false;
    active_samples_.clear();
    last_published_sample_index_ = 0;
  }

  DeltaRobot robot_;
  delta_motion::MotionLimits motion_limits_{
      kDefaultVelocityMmS * mmtm,
      kDefaultAccelerationMmS2 * mmtm,
  };
  Point current_tcp_m_{delta_robot_config::kHomeTcpPositionM};
  sensor_msgs::msg::JointState joint_state_;

  rclcpp::TimerBase::SharedPtr motion_timer_;
  rclcpp::TimerBase::SharedPtr idle_state_timer_;

  bool motion_active_{false};
  std::size_t last_published_sample_index_{0};
  SteadyClock::time_point motion_started_at_{};
  std::string active_motion_description_;
  std::vector<delta_motion::TrajectorySample> active_samples_;

  rclcpp::Subscription<my_delta_robot::msg::LinearSpeedXYZ>::SharedPtr
      segment_subscription_;
  rclcpp::Subscription<my_delta_robot::msg::CircleXYZ>::SharedPtr
      circle_subscription_;
  rclcpp::Subscription<my_delta_robot::msg::VmaxAmax>::SharedPtr
      motion_limits_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_states_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<my_delta_robot::msg::VmaxAmax>::SharedPtr
      profile_publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MainNode>());
  rclcpp::shutdown();
  return 0;
}
