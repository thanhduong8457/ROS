#pragma once

#include "common.h"

#include <algorithm>
#include <cmath>

namespace delta_motion {

struct JogStep {
  Point position_m;
  double speed_mps{0.0};
  double acceleration_mps2{0.0};
};

inline JogStep advanceJog(const Point &current_position_m,
                          const Point &unit_direction, double current_speed_mps,
                          double requested_speed_mps,
                          double max_acceleration_mps2, double elapsed_s) {
  const double next_speed_mps =
      std::min(requested_speed_mps,
               current_speed_mps + max_acceleration_mps2 * elapsed_s);
  const double distance_m =
      0.5 * (current_speed_mps + next_speed_mps) * elapsed_s;

  return {
      {
          current_position_m.x + unit_direction.x * distance_m,
          current_position_m.y + unit_direction.y * distance_m,
          current_position_m.z + unit_direction.z * distance_m,
      },
      next_speed_mps,
      (next_speed_mps - current_speed_mps) / elapsed_s,
  };
}

inline bool isUnitAxisDirection(const Point &direction) {
  if (!std::isfinite(direction.x) || !std::isfinite(direction.y) ||
      !std::isfinite(direction.z)) {
    return false;
  }
  return (std::abs(direction.x) == 1.0 && direction.y == 0.0 &&
          direction.z == 0.0) ||
         (direction.x == 0.0 && std::abs(direction.y) == 1.0 &&
          direction.z == 0.0) ||
         (direction.x == 0.0 && direction.y == 0.0 &&
          std::abs(direction.z) == 1.0);
}

} // namespace delta_motion
