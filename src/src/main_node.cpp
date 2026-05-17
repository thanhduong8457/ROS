#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

#include "delta_robot.h"
#include "joint_state_config.hpp"
#include "my_delta_robot/msg/linear_speed_xyz.hpp"
#include "my_delta_robot/msg/num_point.hpp"
#include "my_delta_robot/msg/vmax_amax.hpp"

namespace {

constexpr double kMinTrajectoryStepSec = 0.01;
constexpr double kTrajectoryTimeScale = 10.0;

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
        using delta_robot_config::kActuatedJoints;
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

        robot_->set_vmax_amax(1500, 200000);
        robot_->set_resolution(120);

        publishInitialJointState();

        // Republish until robot_state_publisher is subscribed (avoids startup race)
        startup_publish_count_ = 0;
        startup_timer_ = create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&MainNode::onStartupTimer, this));
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
        robot_->mStartPoint.x = msg->xo;
        robot_->mStartPoint.y = msg->yo;
        robot_->mStartPoint.z = msg->zo;
        robot_->mEndPoint.x = msg->xf;
        robot_->mEndPoint.y = msg->yf;
        robot_->mEndPoint.z = msg->zf;
        if (robot_->mStartPoint == robot_->mEndPoint) {
            RCLCPP_WARN(get_logger(), "Start and end points are identical; ignoring.");
            return;
        }
        executeTrajectory();
    }

    void onNumPoint(const my_delta_robot::msg::NumPoint::SharedPtr msg) {
        if (msg->resolution <= 0) {
            RCLCPP_ERROR(get_logger(), "Invalid resolution: %lld", static_cast<long long>(msg->resolution));
            return;
        }
        robot_->set_resolution(msg->resolution);
        RCLCPP_INFO(get_logger(), "Resolution set to %lld", static_cast<long long>(msg->resolution));
    }

    void onVmaxAmax(const my_delta_robot::msg::VmaxAmax::SharedPtr msg) {
        robot_->set_vmax_amax(static_cast<unsigned int>(msg->vmax),
                              static_cast<unsigned int>(msg->amax));
        RCLCPP_INFO(get_logger(), "vmax=%.1f amax=%.1f", msg->vmax, msg->amax);
    }

    void executeTrajectory() {
        if (!rclcpp::ok()) {
            return;
        }

        robot_->system_linear();
        robot_->TrapezoidalVelocityProfile();
        robot_->system_linear_matrix();

        if (robot_->m_data_delta.empty()) {
            RCLCPP_WARN(get_logger(), "Trajectory produced no points; skipping.");
            return;
        }

        RCLCPP_INFO(get_logger(), "Publishing linear path to RViz");

        vm_am_.vmax = robot_->m_data_delta[0]->vel;
        vm_am_.amax = robot_->m_data_delta[0]->acel;
        pub_v_a_out_->publish(vm_am_);

        for (size_t i = 1; i < robot_->m_data_delta.size(); ++i) {
            const auto & sample = robot_->m_data_delta[i];
            const auto & prev = robot_->m_data_delta[i - 1];

            sample->theta_val = robot_->inverse(sample->position_val);
            robot_->create_joint_state_list(
                sample->position_val, sample->theta_val, position_value_);

            double delta = (sample->time_point - prev->time_point) * kTrajectoryTimeScale;
            if (delta <= 0.0) {
                delta = kMinTrajectoryStepSec;
            }

            for (int j = 0; j < delta_robot_config::kNumJoints; ++j) {
                joint_state_.position[j] = position_value_[j];
            }
            joint_state_.header.stamp = now();
            pub_joint_states_->publish(joint_state_);

            vm_am_.vmax = sample->vel;
            vm_am_.amax = sample->acel;
            pub_v_a_out_->publish(vm_am_);

            rclcpp::Rate rate(1.0 / delta);
            rate.sleep();
        }

        status_msg_.data =
            "(" + std::to_string(robot_->mStartPoint.x) + ", "
            + std::to_string(robot_->mStartPoint.y) + ", "
            + std::to_string(robot_->mStartPoint.z) + ") -> ("
            + std::to_string(robot_->mEndPoint.x) + ", "
            + std::to_string(robot_->mEndPoint.y) + ", "
            + std::to_string(robot_->mEndPoint.z) + ") DONE";
        pub_status_->publish(status_msg_);
    }

    double position_value_[12];

    sensor_msgs::msg::JointState joint_state_;
    my_delta_robot::msg::VmaxAmax vm_am_;
    std_msgs::msg::String status_msg_;
    std::unique_ptr<delta_robot> robot_;

    int startup_publish_count_{0};
    rclcpp::TimerBase::SharedPtr startup_timer_;

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
