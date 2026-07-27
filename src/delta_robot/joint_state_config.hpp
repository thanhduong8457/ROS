#pragma once

#include "common.h"

#include <cstddef>
#include <string>
#include <vector>

namespace delta_robot_config {

// Joints published to robot_state_publisher (fixed gripper joint omitted).
inline constexpr std::size_t kNumJoints = 12;
inline constexpr Point kHomeTcpPositionM{0.0, 0.0, -0.375};

inline const std::vector<std::string> kJointNames = {
    "base_brazo1", "base_brazo2", "base_brazo3", "codo1_a",
    "codo1_b",     "codo2_a",     "codo2_b",     "codo3_a",
    "codo3_b",     "act_x",       "act_y",       "act_z",
};

} // namespace delta_robot_config
