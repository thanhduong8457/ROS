#pragma once

#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace delta_robot_config {

// Joints published to robot_state_publisher (fixed gripper joint omitted).
inline constexpr int kNumJoints = 12;
inline constexpr int kActuatedJoints = 12;

inline const std::vector<std::string> kJointNames = {
    "base_brazo1", "base_brazo2", "base_brazo3",
    "codo1_a", "codo1_b", "codo2_a", "codo2_b", "codo3_a", "codo3_b",
    "act_x", "act_y", "act_z",
};

/// TCP in base_link frame [m]: X/Y horizontal, Z up (workspace Z is negative = down).
inline void mapTcpToActuatorJoints(
    double x_m, double y_m, double z_m,
    double & act_x, double & act_y, double & act_z)
{
    act_x = x_m;
    act_y = y_m;
    act_z = z_m;
}

/// Home pose: (0, 0, -375 mm) — must match inverse kinematics at startup.
inline const std::array<double, kNumJoints> kInitialJointPositions = {
    0.000266315, 0.000266533, 0.000266315,
    0.9382, 7.38599e-08, 0.9382, -1.32324e-11, 0.9382, -7.38454e-08,
    0.0, 0.0, -0.375,
};

}  // namespace delta_robot_config
