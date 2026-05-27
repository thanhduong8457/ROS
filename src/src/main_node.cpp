#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

#include "delta_robot.h"
#include "joint_state_config.hpp"
#include "motion_planner.hpp"
#include "my_delta_robot/msg/linear_speed_xyz.hpp"
#include "my_delta_robot/msg/num_point.hpp"
#include "my_delta_robot/msg/vmax_amax.hpp"

namespace {

constexpr auto kMotionTick = std::chrono::milliseconds(1);
constexpr double kMotionSamplePeriodSec = 0.001;

}  // namespace

class MainNode : public rclcpp::Node {
public:
    MainNode()
    : Node("main_node"),
      position_value_{},
      joint_state_(),
      vm_am_(),
      status_msg_(),
      robot_(std::make_unique<delta_robot>()) {
        using delta_robot_config::kJointNames;
        using delta_robot_config::kNumJoints;

        RCLCPP_INFO(get_logger(), "main_node started");

        sub_linear_speed_ = create_subscription<my_delta_robot::msg::LinearSpeedXYZ>(
            "input_ls_final", 10,
            std::bind(&MainNode::onLinearSpeed, this, std::placeholders::_1));
        sub_num_point_ = create_subscription<my_delta_robot::msg::NumPoint>(
            "set_num_point", 10,
            std::bind(&MainNode::onNumPoint, this, std::placeholders::_1));
        sub_vmax_amax_ = create_subscription<my_delta_robot::msg::VmaxAmax>(
            "set_vmax_amax", 10,
            std::bind(&MainNode::onVmaxAmax, this, std::placeholders::_1));

        pub_joint_states_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        pub_status_ = create_publisher<std_msgs::msg::String>("status_delta", 10);
        pub_v_a_out_ = create_publisher<my_delta_robot::msg::VmaxAmax>("v_a_out", 10);

        joint_state_.header.frame_id = "base_link";
        joint_state_.name = kJointNames;
        joint_state_.position.resize(kNumJoints, 0.0);

        robot_->set_vmax_amax(5000, 100);
        robot_->set_resolution(120);

        publishInitialJointState();

        // Republish until robot_state_publisher is subscribed (avoids startup race)
        startup_publish_count_ = 0;
        startup_timer_ = create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&MainNode::onStartupTimer, this));

        motion_timer_ = create_wall_timer(
            kMotionTick,
            std::bind(&MainNode::onMotionTimer, this));
    }

private:
    void onStartupTimer() {
        publishInitialJointState();
        ++startup_publish_count_;
        if (startup_publish_count_ >= 15) {
            startup_timer_->cancel();
            startup_timer_.reset();
        }
    }

    void publishInitialJointState() {
        using delta_robot_config::kInitialJointPositions;
        using delta_robot_config::kNumJoints;

        for (int i = 0; i < kNumJoints; ++i) {
            joint_state_.position[i] = kInitialJointPositions[i];
        }
        joint_state_.header.stamp = now();
        pub_joint_states_->publish(joint_state_);
    }

    void onLinearSpeed(const my_delta_robot::msg::LinearSpeedXYZ::SharedPtr msg) {
        if (motion_active_) {
            RCLCPP_WARN(get_logger(), "Motion already active; rejecting new command.");
            return;
        }

        commanded_start_mm_ = Point(msg->xo, msg->yo, msg->zo);
        commanded_target_mm_ = Point(msg->xf, msg->yf, msg->zf);
        if (commanded_start_mm_ == commanded_target_mm_) {
            RCLCPP_WARN(get_logger(), "Start and end points are identical; ignoring.");
            return;
        }
        startTrajectory(commanded_start_mm_, commanded_target_mm_);
    }

    void onNumPoint(const my_delta_robot::msg::NumPoint::SharedPtr msg) {
        if (msg->resolution <= 0) {
            RCLCPP_ERROR(get_logger(), "Invalid resolution: %lld", static_cast<long long>(msg->resolution));
            return;
        }
        robot_->set_resolution(static_cast<unsigned int>(msg->resolution));
        RCLCPP_INFO(
            get_logger(),
            "Legacy offline resolution set to %lld; runtime planner remains fixed at 1 kHz.",
            static_cast<long long>(msg->resolution));
    }

    void onVmaxAmax(const my_delta_robot::msg::VmaxAmax::SharedPtr msg) {
        robot_->set_vmax_amax(static_cast<unsigned int>(msg->vmax),
                              static_cast<unsigned int>(msg->amax));
        RCLCPP_INFO(get_logger(), "vmax=%.1f amax=%.1f", msg->vmax, msg->amax);
    }

    void startTrajectory(const Point& start_mm, const Point& target_mm) {
        Point start_m(start_mm.x * mmtm, start_mm.y * mmtm, start_mm.z * mmtm);
        Point target_m(target_mm.x * mmtm, target_mm.y * mmtm, target_mm.z * mmtm);

        delta_motion::CartesianTrajectoryGenerator generator;
        auto plan = generator.planLine(
            start_m, target_m, robot_->motion_limits(), kMotionSamplePeriodSec);
        if (!plan.ok) {
            RCLCPP_ERROR(get_logger(), "Trajectory rejected: %s", plan.error.c_str());
            return;
        }

        for (const auto& sample : plan.samples) {
            const auto ik = robot_->inverse_checked(sample.position_m);
            if (!ik.ok) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Trajectory rejected at t=%.4f s: %s",
                    sample.time_s,
                    ik.error.c_str());
                return;
            }
        }

        active_samples_ = std::move(plan.samples);
        active_sample_index_ = 0;
        motion_active_ = true;

        RCLCPP_INFO(
            get_logger(),
            "Started Cartesian line: %zu samples, %.4f s",
            active_samples_.size(),
            active_samples_.back().time_s);
    }

    void onMotionTimer() {
        if (!motion_active_ || active_sample_index_ >= active_samples_.size()) {
            return;
        }

        const auto& sample = active_samples_[active_sample_index_];
        const auto ik = robot_->inverse_checked(sample.position_m);
        if (!ik.ok) {
            RCLCPP_ERROR(get_logger(), "IK failed during motion: %s", ik.error.c_str());
            motion_active_ = false;
            active_samples_.clear();
            return;
        }

        robot_->create_joint_state_list(sample.position_m, ik.theta, position_value_);

        for (int j = 0; j < delta_robot_config::kNumJoints; ++j) {
            joint_state_.position[j] = position_value_[j];
        }
        joint_state_.header.stamp = now();
        pub_joint_states_->publish(joint_state_);

        vm_am_.vmax = sample.path_velocity_mps;
        vm_am_.amax = sample.path_acceleration_mps2;
        pub_v_a_out_->publish(vm_am_);

        ++active_sample_index_;
        if (active_sample_index_ < active_samples_.size()) {
            return;
        }

        status_msg_.data =
            "(" + std::to_string(commanded_start_mm_.x) + ", "
            + std::to_string(commanded_start_mm_.y) + ", "
            + std::to_string(commanded_start_mm_.z) + ") -> ("
            + std::to_string(commanded_target_mm_.x) + ", "
            + std::to_string(commanded_target_mm_.y) + ", "
            + std::to_string(commanded_target_mm_.z) + ") DONE";
        pub_status_->publish(status_msg_);
        motion_active_ = false;
        active_samples_.clear();
    }

    double position_value_[12];

    sensor_msgs::msg::JointState joint_state_;
    my_delta_robot::msg::VmaxAmax vm_am_;
    std_msgs::msg::String status_msg_;
    std::unique_ptr<delta_robot> robot_;

    int startup_publish_count_{0};
    rclcpp::TimerBase::SharedPtr startup_timer_;
    rclcpp::TimerBase::SharedPtr motion_timer_;

    bool motion_active_{false};
    size_t active_sample_index_{0};
    Point commanded_start_mm_;
    Point commanded_target_mm_;
    std::vector<delta_motion::TrajectorySample> active_samples_;

    rclcpp::Subscription<my_delta_robot::msg::LinearSpeedXYZ>::SharedPtr sub_linear_speed_;
    rclcpp::Subscription<my_delta_robot::msg::NumPoint>::SharedPtr sub_num_point_;
    rclcpp::Subscription<my_delta_robot::msg::VmaxAmax>::SharedPtr sub_vmax_amax_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Publisher<my_delta_robot::msg::VmaxAmax>::SharedPtr pub_v_a_out_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainNode>());
    rclcpp::shutdown();
    return 0;
}
