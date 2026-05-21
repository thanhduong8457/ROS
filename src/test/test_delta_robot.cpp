/**
 * Unit test for delta_robot kinematics (inverse and trajectory).
 */
#include <gtest/gtest.h>
#include "delta_robot.h"
#include "motion_planner.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DTR (M_PI / 180.0)

TEST(DeltaRobot, InverseAtOrigin) {
  delta_robot robot;
  robot.initialize();
  Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = -0.375;  // within workspace
  Theta theta = robot.inverse(p);
  EXPECT_FALSE(std::isnan(theta.angle1));
  EXPECT_FALSE(std::isnan(theta.angle2));
  EXPECT_FALSE(std::isnan(theta.angle3));
}

TEST(DeltaRobot, CheckedInverseRejectsInvalidPoint) {
  delta_robot robot;
  Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = 0.0;
  auto result = robot.inverse_checked(p);
  EXPECT_FALSE(result.ok);
}

TEST(DeltaRobot, ActuatorJointsAtHomePose) {
  delta_robot robot;
  Point tcp;
  tcp.x = 0.0;
  tcp.y = 0.0;
  tcp.z = -0.375;
  Theta theta = robot.inverse(tcp);
  double joints[12] = {};
  robot.create_joint_state_list(tcp, theta, joints);
  EXPECT_NEAR(joints[9], 0.0, 1e-6);
  EXPECT_NEAR(joints[10], 0.0, 1e-6);
  EXPECT_NEAR(joints[11], -0.375, 1e-4);
}

TEST(DeltaRobot, SetVmaxAmax) {
  delta_robot robot;
  robot.set_vmax_amax(1000, 100000);
  robot.set_resolution(50);
  Point start, end;
  start.x = 0.0;
  start.y = 0.0;
  start.z = -375.0;
  end.x = 0.0;
  end.y = 0.0;
  end.z = -400.0;
  robot.mStartPoint = start;
  robot.mEndPoint = end;
  robot.system_linear();
  robot.TrapezoidalVelocityProfile();
  robot.system_linear_matrix();
  EXPECT_GT(robot.m_data_delta.size(), 0u);
}

TEST(CartesianTrajectory, SamplesStraightLineWithTrapezoidState) {
  delta_motion::CartesianTrajectoryGenerator generator;
  delta_motion::MotionLimits limits;
  limits.max_velocity_mps = 0.5;
  limits.max_acceleration_mps2 = 5.0;

  Point start(0.0, 0.0, -0.375);
  Point target(0.05, 0.025, -0.425);
  auto plan = generator.planLine(start, target, limits, 0.001);

  ASSERT_TRUE(plan.ok) << plan.error;
  ASSERT_GT(plan.samples.size(), 2u);
  EXPECT_NEAR(plan.samples.front().position_m.x, start.x, 1e-9);
  EXPECT_NEAR(plan.samples.front().position_m.y, start.y, 1e-9);
  EXPECT_NEAR(plan.samples.front().position_m.z, start.z, 1e-9);
  EXPECT_NEAR(plan.samples.back().position_m.x, target.x, 1e-9);
  EXPECT_NEAR(plan.samples.back().position_m.y, target.y, 1e-9);
  EXPECT_NEAR(plan.samples.back().position_m.z, target.z, 1e-9);
  EXPECT_NEAR(plan.samples.back().path_velocity_mps, 0.0, 1e-9);

  const double dx = target.x - start.x;
  const double dy = target.y - start.y;
  const double dz = target.z - start.z;
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

  for (const auto& sample : plan.samples) {
    const double s = sample.path_position_m / distance;
    EXPECT_NEAR(sample.position_m.x, start.x + dx * s, 1e-8);
    EXPECT_NEAR(sample.position_m.y, start.y + dy * s, 1e-8);
    EXPECT_NEAR(sample.position_m.z, start.z + dz * s, 1e-8);
    EXPECT_LE(std::abs(sample.path_velocity_mps), limits.max_velocity_mps + 1e-9);
  }
}
