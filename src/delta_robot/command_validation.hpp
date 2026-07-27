#pragma once

#include "common.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>

namespace delta_robot_validation {

inline bool isFinitePoint(const Point &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

inline bool pointsNear(const Point &lhs, const Point &rhs, double tolerance) {
  return std::isfinite(tolerance) && tolerance >= 0.0 && isFinitePoint(lhs) &&
         isFinitePoint(rhs) && std::abs(lhs.x - rhs.x) <= tolerance &&
         std::abs(lhs.y - rhs.y) <= tolerance &&
         std::abs(lhs.z - rhs.z) <= tolerance;
}

template <std::size_t Size>
inline bool areFinite(const std::array<double, Size> &values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
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
