#include "delta_robot.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kSingularityTolerance = 1e-12;
constexpr double kDiscriminantTolerance = 1e-12;

bool isFinite(const Point &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

bool isFinite(const Theta &theta) {
  return std::isfinite(theta.angle1) && std::isfinite(theta.angle2) &&
         std::isfinite(theta.angle3);
}

} // namespace

DeltaRobot::IkResult DeltaRobot::inverseChecked(const Point &point_m) const {
  IkResult result;
  if (!isFinite(point_m)) {
    result.error = "IK point coordinates must be finite";
    return result;
  }

  Theta theta;
  Point rotated_point;

  if (!angleYz(point_m, theta.angle2)) {
    result.error = "IK failed for arm 2";
    return result;
  }

  rotated_point.x = point_m.x * cos120 + point_m.y * sin120;
  rotated_point.y = point_m.y * cos120 - point_m.x * sin120;
  rotated_point.z = point_m.z;
  if (!angleYz(rotated_point, theta.angle3)) {
    result.error = "IK failed for arm 3";
    return result;
  }

  rotated_point.x = point_m.x * cos120 - point_m.y * sin120;
  rotated_point.y = point_m.y * cos120 + point_m.x * sin120;
  rotated_point.z = point_m.z;
  if (!angleYz(rotated_point, theta.angle1)) {
    result.error = "IK failed for arm 1";
    return result;
  }

  result.ok = true;
  result.theta = theta;
  return result;
}

bool DeltaRobot::angleYz(Point point_m, double &theta_deg) {
  theta_deg = 0.0;
  if (!isFinite(point_m) || std::abs(point_m.z) < kSingularityTolerance) {
    return false;
  }

  const double base_joint_y_m = -0.5 * tan30 * ff * mmtm;
  point_m.y -= 0.5 * tan30 * ee * mmtm;

  const double upper_arm_m = rf * mmtm;
  const double forearm_m = re * mmtm;
  const double line_offset =
      (point_m.x * point_m.x + point_m.y * point_m.y + point_m.z * point_m.z +
       upper_arm_m * upper_arm_m - forearm_m * forearm_m -
       base_joint_y_m * base_joint_y_m) /
      (2.0 * point_m.z);
  const double line_slope = (base_joint_y_m - point_m.y) / point_m.z;
  double discriminant =
      -(line_offset + line_slope * base_joint_y_m) *
          (line_offset + line_slope * base_joint_y_m) +
      upper_arm_m * upper_arm_m * (line_slope * line_slope + 1.0);

  if (!std::isfinite(discriminant) || discriminant < -kDiscriminantTolerance) {
    return false;
  }
  discriminant = std::max(0.0, discriminant);

  const double elbow_y =
      (base_joint_y_m - line_offset * line_slope - std::sqrt(discriminant)) /
      (line_slope * line_slope + 1.0);
  const double elbow_z = line_offset + line_slope * elbow_y;
  theta_deg = std::atan2(-elbow_z, base_joint_y_m - elbow_y) * rtd;
  return std::isfinite(theta_deg);
}

bool DeltaRobot::createJointStatePositions(
    const Point &point_m, const Theta &theta_deg,
    JointPositions &positions_rad_m) const {
  if (!isFinite(point_m) || !isFinite(theta_deg)) {
    return false;
  }

  // RViz uses the model's axis convention, which swaps and negates TCP X/Y/Z.
  const Vector3 tcp_position{-point_m.y, -point_m.x, -point_m.z};

  const Vector3 elbow1 = rotate240(elbowPoint(theta_deg.angle1));
  const Vector3 elbow2 = elbowPoint(theta_deg.angle2);
  const Vector3 elbow3 = rotate120(elbowPoint(theta_deg.angle3));
  const Vector3 end_effector1 = endEffectorPoint(Arm::One, tcp_position);
  const Vector3 end_effector2 = endEffectorPoint(Arm::Two, tcp_position);
  const Vector3 end_effector3 = endEffectorPoint(Arm::Three, tcp_position);

  double arm1_angle_a = 0.0;
  double arm1_angle_b = 0.0;
  double arm2_angle_a = 0.0;
  double arm2_angle_b = 0.0;
  double arm3_angle_a = 0.0;
  double arm3_angle_b = 0.0;
  if (!elbowAngles(Arm::One, elbow1, end_effector1, arm1_angle_a,
                   arm1_angle_b) ||
      !elbowAngles(Arm::Two, elbow2, end_effector2, arm2_angle_a,
                   arm2_angle_b) ||
      !elbowAngles(Arm::Three, elbow3, end_effector3, arm3_angle_a,
                   arm3_angle_b)) {
    return false;
  }

  JointPositions positions{};
  positions[0] = theta_deg.angle1 * dtr;
  positions[1] = theta_deg.angle2 * dtr;
  positions[2] = theta_deg.angle3 * dtr;
  positions[3] = positions[0] + arm1_angle_a;
  positions[4] = arm1_angle_b;
  positions[5] = positions[1] + arm2_angle_a;
  positions[6] = arm2_angle_b;
  positions[7] = positions[2] + arm3_angle_a;
  positions[8] = arm3_angle_b;
  positions[9] = point_m.x;
  positions[10] = point_m.y;
  positions[11] = point_m.z;

  if (!std::all_of(positions.begin(), positions.end(),
                   [](double value) { return std::isfinite(value); })) {
    return false;
  }

  positions_rad_m = positions;
  return true;
}

DeltaRobot::Vector3 DeltaRobot::elbowPoint(double theta_deg) {
  const double theta_rad = theta_deg * dtr;
  return {(hf * mmtm) / 3.0 + rf * mmtm * std::cos(theta_rad), 0.0,
          rf * mmtm * std::sin(theta_rad)};
}

DeltaRobot::Vector3 DeltaRobot::rotate120(const Vector3 &input) {
  return {cos120 * input[0] + sin120 * input[1],
          -sin120 * input[0] + cos120 * input[1], input[2]};
}

DeltaRobot::Vector3 DeltaRobot::rotate240(const Vector3 &input) {
  return {cos240 * input[0] + sin240 * input[1],
          -sin240 * input[0] + cos240 * input[1], input[2]};
}

DeltaRobot::Vector3 DeltaRobot::rotateY(const Vector3 &input,
                                        double angle_rad) {
  const double cosine = std::cos(angle_rad);
  const double sine = std::sin(angle_rad);
  return {input[0] * cosine - input[2] * sine, input[1],
          -input[0] * sine + input[2] * cosine};
}

DeltaRobot::Vector3 DeltaRobot::endEffectorPoint(Arm arm,
                                                 const Vector3 &tcp_position) {
  const Vector3 arm2_offset{(he * mmtm) / 3.0, 0.0, 0.0};
  const Vector3 arm3_offset = rotate120(arm2_offset);
  const Vector3 arm1_offset = rotate120(arm3_offset);

  Vector3 offset = arm2_offset;
  if (arm == Arm::One) {
    offset = arm1_offset;
  } else if (arm == Arm::Three) {
    offset = arm3_offset;
  }

  return {tcp_position[0] + offset[0], tcp_position[1] + offset[1],
          tcp_position[2] + offset[2]};
}

bool DeltaRobot::elbowAngles(Arm arm, const Vector3 &elbow,
                             const Vector3 &end_effector, double &angle_a_rad,
                             double &angle_b_rad) {
  Vector3 oriented_elbow = elbow;
  Vector3 oriented_end_effector = end_effector;
  if (arm == Arm::One) {
    oriented_elbow = rotate120(elbow);
    oriented_end_effector = rotate120(end_effector);
  } else if (arm == Arm::Three) {
    oriented_elbow = rotate240(elbow);
    oriented_end_effector = rotate240(end_effector);
  }

  angle_a_rad = std::atan2(oriented_end_effector[2] - oriented_elbow[2],
                           oriented_elbow[0] - oriented_end_effector[0]);

  // Value-returning transforms preserve both source coordinates.
  oriented_elbow = rotateY(oriented_elbow, angle_a_rad);
  oriented_end_effector = rotateY(oriented_end_effector, angle_a_rad);
  angle_b_rad = std::atan2(oriented_end_effector[1],
                           oriented_elbow[0] - oriented_end_effector[0]);

  return std::isfinite(angle_a_rad) && std::isfinite(angle_b_rad);
}
