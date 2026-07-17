#pragma once

#include "common.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace delta_drawing {

inline bool pointsNear(const Point &lhs, const Point &rhs,
                       double tolerance_mm = 1e-6) {
  return std::abs(lhs.x - rhs.x) <= tolerance_mm &&
         std::abs(lhs.y - rhs.y) <= tolerance_mm &&
         std::abs(lhs.z - rhs.z) <= tolerance_mm;
}

inline Point withDrawOffset(const Point &point, double z_offset_mm) {
  return {point.x, point.y, point.z - z_offset_mm};
}

inline std::vector<Point> rectanglePath(const std::array<Point, 4> &corners,
                                        double z_offset_mm,
                                        const Point &return_point) {
  std::vector<Point> path;
  path.reserve(6);
  for (const auto &corner : corners) {
    path.push_back(withDrawOffset(corner, z_offset_mm));
  }
  path.push_back(withDrawOffset(corners.front(), z_offset_mm));
  path.push_back(return_point);
  return path;
}

inline std::vector<Point> trianglePath(const std::array<Point, 4> &corners,
                                       double z_offset_mm,
                                       const Point &return_point) {
  std::vector<Point> path;
  path.reserve(5);
  for (std::size_t i = 0; i < 3; ++i) {
    path.push_back(withDrawOffset(corners[i], z_offset_mm));
  }
  path.push_back(withDrawOffset(corners.front(), z_offset_mm));
  path.push_back(return_point);
  return path;
}

inline std::vector<Point> circlePath(const Point &center, double radius_mm,
                                     double z_offset_mm,
                                     std::size_t segment_count,
                                     const Point &return_point) {
  if (!std::isfinite(radius_mm) || radius_mm <= 0.0 || segment_count < 3) {
    return {};
  }

  std::vector<Point> path;
  path.reserve(segment_count + 2);
  const double draw_z = center.z - z_offset_mm;
  constexpr double kTwoPi = 6.28318530717958647692;
  for (std::size_t i = 0; i < segment_count; ++i) {
    const double angle =
        kTwoPi * static_cast<double>(i) / static_cast<double>(segment_count);
    path.emplace_back(center.x + radius_mm * std::cos(angle),
                      center.y + radius_mm * std::sin(angle), draw_z);
  }
  path.push_back(path.front());
  path.push_back(return_point);
  return path;
}

} // namespace delta_drawing
