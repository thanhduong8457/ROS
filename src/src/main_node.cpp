#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
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
#include "my_delta_robot/msg/num_point.hpp"
#include "my_delta_robot/msg/vmax_amax.hpp"

namespace {

using SteadyClock = std::chrono::steady_clock;

constexpr auto kMotionTick = std::chrono::milliseconds(1);
constexpr auto kIdleStatePublishPeriod = std::chrono::milliseconds(200);
constexpr double kMotionSamplePeriodSec = 0.001;
constexpr double kDefaultVelocityMmS = 5000.0;
constexpr double kDefaultAccelerationMmS2 = 100.0;
constexpr unsigned int kDefaultLegacyResolution = 120;

Point millimetresToMetres(const Point &point_mm) {
  return {point_mm.x * mmtm, point_mm.y * mmtm, point_mm.z * mmtm};
}

} // namespace

class MainNode : public rclcpp::Node {
public:
  MainNode() : Node("main_node") {
    RCLCPP_INFO(get_logger(), "main_node started");

    segment_subscription_ =
        create_subscription<my_delta_robot::msg::LinearSpeedXYZ>(
            "input_ls_final", 10,
            std::bind(&MainNode::onSegmentCommand, this,
                      std::placeholders::_1));
    circle_subscription_ = create_subscription<my_delta_robot::msg::CircleXYZ>(
        "input_circle", 10,
        std::bind(&MainNode::onCircleCommand, this, std::placeholders::_1));
    resolution_subscription_ =
        create_subscription<my_delta_robot::msg::NumPoint>(
            "set_num_point", 10,
            std::bind(&MainNode::onResolutionCommand, this,
                      std::placeholders::_1));
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
    std::copy(delta_robot_config::kInitialJointPositions.begin(),
              delta_robot_config::kInitialJointPositions.end(),
              joint_state_.position.begin());

    robot_.set_vmax_amax(kDefaultVelocityMmS, kDefaultAccelerationMmS2);
    robot_.set_resolution(kDefaultLegacyResolution);

    publishCurrentJointState();
    motion_timer_ = create_wall_timer(
        kMotionTick, std::bind(&MainNode::onMotionTimer, this));
    idle_state_timer_ = create_wall_timer(
        kIdleStatePublishPeriod, std::bind(&MainNode::onIdleStateTimer, this));
  }

private:
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

    const Point start_mm(msg->xo, msg->yo, msg->zo);
    const Point target_mm(msg->xf, msg->yf, msg->zf);
    if (!delta_robot_validation::isFinitePoint(start_mm) ||
        !delta_robot_validation::isFinitePoint(target_mm)) {
      publishFailure("Segment coordinates must be finite");
      return;
    }
    if (start_mm == target_mm) {
      publishFailure("Start and end points are identical");
      return;
    }

    startTrajectory(start_mm, target_mm);
  }

  void onResolutionCommand(const my_delta_robot::msg::NumPoint::SharedPtr msg) {
    if (msg->resolution <= 0 ||
        static_cast<unsigned long long>(msg->resolution) >
            std::numeric_limits<unsigned int>::max()) {
      RCLCPP_ERROR(get_logger(), "Invalid resolution: %lld",
                   static_cast<long long>(msg->resolution));
      return;
    }
    robot_.set_resolution(static_cast<unsigned int>(msg->resolution));
    RCLCPP_INFO(get_logger(),
                "Legacy offline resolution set to %lld; runtime planner "
                "remains fixed at 1 kHz.",
                static_cast<long long>(msg->resolution));
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

    delta_motion::CartesianTrajectoryGenerator generator;
    auto plan = generator.planCircle(
        millimetresToMetres(center_mm), msg->radius * mmtm, msg->clockwise,
        robot_.motion_limits(), kMotionSamplePeriodSec);
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

    robot_.set_vmax_amax(msg->vmax, msg->amax);
    RCLCPP_INFO(get_logger(),
                "Motion limits updated: vmax=%.1f mm/s, amax=%.1f mm/s^2",
                msg->vmax, msg->amax);
  }

  void startTrajectory(const Point &start_mm, const Point &target_mm) {
    delta_motion::CartesianTrajectoryGenerator generator;
    auto plan = generator.planLine(
        millimetresToMetres(start_mm), millimetresToMetres(target_mm),
        robot_.motion_limits(), kMotionSamplePeriodSec);
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

    for (const auto &sample : plan.samples) {
      const auto ik = robot_.inverse_checked(sample.position_m);
      if (!ik.ok) {
        publishFailure("Trajectory rejected at t=" +
                       std::to_string(sample.time_s) + " s: " + ik.error);
        return;
      }
    }

    active_motion_description_ = std::move(description);
    active_samples_ = std::move(plan.samples);
    last_published_sample_index_ = 0;
    motion_started_at_ = SteadyClock::now();
    motion_active_ = true;
    publishSample(active_samples_.front());

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
      publishFailure("IK failed during motion");
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
    const auto ik = robot_.inverse_checked(sample.position_m);
    if (!ik.ok) {
      RCLCPP_ERROR(get_logger(), "IK failed during motion: %s",
                   ik.error.c_str());
      return false;
    }

    std::array<double, delta_robot_config::kNumJoints> joint_positions{};
    robot_.create_joint_state_list(sample.position_m, ik.theta,
                                   joint_positions);
    std::copy(joint_positions.begin(), joint_positions.end(),
              joint_state_.position.begin());
    joint_state_.header.stamp = now();
    joint_states_publisher_->publish(joint_state_);

    my_delta_robot::msg::VmaxAmax profile;
    profile.vmax = sample.path_velocity_mps;
    profile.amax = sample.path_acceleration_mps2;
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

  delta_robot robot_;
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
  rclcpp::Subscription<my_delta_robot::msg::NumPoint>::SharedPtr
      resolution_subscription_;
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
