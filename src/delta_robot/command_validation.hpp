#pragma once

#include "common.h"

#include <cmath>
#include <string>

namespace delta_robot_validation {

inline bool isFinitePoint(const Point &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

inline std::string validateMotionLimits(double velocity_mm_s,
                                        double acceleration_mm_s2) {
  if (!std::isfinite(velocity_mm_s) || !std::isfinite(acceleration_mm_s2)) {
    return "Velocity and acceleration must be finite";
  }
  if (velocity_mm_s <= 0.0 || acceleration_mm_s2 <= 0.0) {
    return "Velocity and acceleration must be positive";
  }
  return {};
}

} // namespace delta_robot_validation
