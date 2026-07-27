#pragma once

#include "common.h"

#include <array>
#include <cstddef>
#include <string>

class DeltaRobot {
public:
  static constexpr std::size_t kJointCount = 12;
  using JointPositions = std::array<double, kJointCount>;

  struct IkResult {
    bool ok{false};
    Theta theta;
    std::string error;
  };

  [[nodiscard]] IkResult inverseChecked(const Point &point_m) const;

  [[nodiscard]] bool
  createJointStatePositions(const Point &point_m, const Theta &theta_deg,
                            JointPositions &positions_rad_m) const;

private:
  using Vector3 = std::array<double, 3>;
  enum class Arm { One, Two, Three };

  static bool angleYz(Point point_m, double &theta_deg);
  static Vector3 elbowPoint(double theta_deg);
  static Vector3 rotate120(const Vector3 &input);
  static Vector3 rotate240(const Vector3 &input);
  static Vector3 rotateY(const Vector3 &input, double angle_rad);
  static Vector3 endEffectorPoint(Arm arm, const Vector3 &tcp_position);
  static bool elbowAngles(Arm arm, const Vector3 &elbow,
                          const Vector3 &end_effector, double &angle_a_rad,
                          double &angle_b_rad);
};
