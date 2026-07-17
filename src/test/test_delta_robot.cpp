/**
 * Unit test for delta_robot kinematics (inverse and trajectory).
 */
#include "command_validation.hpp"
#include "delta_robot.h"
#include "motion_planner.hpp"
#include "shape_path.hpp"
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>

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
  p.z = -0.375; // within workspace
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
  std::array<double, 12> joints{};
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

  for (const auto &sample : plan.samples) {
    const double s = sample.path_position_m / distance;
    EXPECT_NEAR(sample.position_m.x, start.x + dx * s, 1e-8);
    EXPECT_NEAR(sample.position_m.y, start.y + dy * s, 1e-8);
    EXPECT_NEAR(sample.position_m.z, start.z + dz * s, 1e-8);
    EXPECT_LE(std::abs(sample.path_velocity_mps),
              limits.max_velocity_mps + 1e-9);
  }
}

TEST(CartesianTrajectory, UsesTriangularProfileForShortMove) {
  delta_motion::CartesianTrajectoryGenerator generator;
  delta_motion::MotionLimits limits{2.0, 1.0};

  const auto plan = generator.planLine(Point(0.0, 0.0, -0.375),
                                       Point(0.01, 0.0, -0.375), limits, 0.001);

  ASSERT_TRUE(plan.ok) << plan.error;
  EXPECT_TRUE(plan.triangular);
  EXPECT_GT(plan.duration_s, 0.0);
  EXPECT_NEAR(plan.samples.back().position_m.x, 0.01, 1e-12);
  EXPECT_DOUBLE_EQ(plan.samples.back().path_velocity_mps, 0.0);
}

TEST(CartesianTrajectory, RejectsNonFiniteInputAndExcessiveSampleCount) {
  delta_motion::CartesianTrajectoryGenerator generator;
  delta_motion::MotionLimits limits{1.0, 1.0};
  const double nan = std::numeric_limits<double>::quiet_NaN();

  const auto non_finite = generator.planLine(
      Point(nan, 0.0, 0.0), Point(1.0, 0.0, 0.0), limits, 0.001);
  EXPECT_FALSE(non_finite.ok);
  EXPECT_EQ(non_finite.error, "Start and target coordinates must be finite");

  const auto excessive = generator.planLine(Point(0.0, 0.0, 0.0),
                                            Point(1.0, 0.0, 0.0), limits, 1e-9);
  EXPECT_FALSE(excessive.ok);
  EXPECT_EQ(excessive.error, "Trajectory requires too many samples");
}

TEST(CartesianTrajectory, SamplesContinuousCircleWithTangentVelocity) {
  delta_motion::CartesianTrajectoryGenerator generator;
  const delta_motion::MotionLimits limits{0.25, 1.0};
  const Point center(-0.1, -0.1, -0.473);
  constexpr double radius = 0.025;

  const auto plan = generator.planCircle(center, radius, false, limits, 0.001);

  ASSERT_TRUE(plan.ok) << plan.error;
  ASSERT_GT(plan.samples.size(), 100u);
  EXPECT_NEAR(plan.samples.front().position_m.x, center.x + radius, 1e-12);
  EXPECT_NEAR(plan.samples.front().position_m.y, center.y, 1e-12);
  EXPECT_NEAR(plan.samples.back().position_m.x,
              plan.samples.front().position_m.x, 1e-12);
  EXPECT_NEAR(plan.samples.back().position_m.y,
              plan.samples.front().position_m.y, 1e-12);

  for (const auto &sample : plan.samples) {
    const double dx = sample.position_m.x - center.x;
    const double dy = sample.position_m.y - center.y;
    EXPECT_NEAR(std::hypot(dx, dy), radius, 1e-10);
    EXPECT_NEAR(sample.position_m.z, center.z, 1e-12);
    EXPECT_NEAR(dx * sample.velocity_mps.x + dy * sample.velocity_mps.y, 0.0,
                1e-10);
    EXPECT_LE(
        std::hypot(sample.acceleration_mps2.x, sample.acceleration_mps2.y),
        limits.max_acceleration_mps2 + 1e-9);
  }

  const auto &moving_sample = plan.samples[plan.samples.size() / 2];
  EXPECT_GT(
      std::hypot(moving_sample.velocity_mps.x, moving_sample.velocity_mps.y),
      0.0);
}

TEST(CartesianTrajectory, SupportsClockwiseCircleAndRejectsInvalidRadius) {
  delta_motion::CartesianTrajectoryGenerator generator;
  const delta_motion::MotionLimits limits{0.25, 1.0};
  const Point center(0.0, 0.0, -0.473);

  const auto clockwise =
      generator.planCircle(center, 0.025, true, limits, 0.001);
  ASSERT_TRUE(clockwise.ok) << clockwise.error;
  ASSERT_GT(clockwise.samples.size(), 2u);
  EXPECT_LT(clockwise.samples[1].velocity_mps.y, 0.0);

  const auto invalid = generator.planCircle(center, 0.0, false, limits, 0.001);
  EXPECT_FALSE(invalid.ok);
  EXPECT_EQ(invalid.error, "Circle radius must be positive");
}

TEST(CommandValidation, RejectsInvalidLimitsAndCoordinates) {
  const double infinity = std::numeric_limits<double>::infinity();

  EXPECT_TRUE(
      delta_robot_validation::validateMotionLimits(5000.5, 100.25).empty());
  EXPECT_FALSE(
      delta_robot_validation::validateMotionLimits(0.0, 100.0).empty());
  EXPECT_FALSE(
      delta_robot_validation::validateMotionLimits(5000.0, -1.0).empty());
  EXPECT_FALSE(
      delta_robot_validation::validateMotionLimits(infinity, 100.0).empty());
  EXPECT_TRUE(delta_robot_validation::isFinitePoint(Point(0.0, 0.0, -0.375)));
  EXPECT_FALSE(
      delta_robot_validation::isFinitePoint(Point(0.0, infinity, 0.0)));
}

TEST(ShapePath, RectangleAndTriangleCloseBeforeReturningHome) {
  const std::array<Point, 4> corners = {
      Point{0.0, 10.0, -453.0},
      Point{-10.0, 0.0, -453.0},
      Point{0.0, -10.0, -453.0},
      Point{10.0, 0.0, -453.0},
  };
  const Point home(0.0, 0.0, -375.0);

  const auto rectangle = delta_drawing::rectanglePath(corners, 20.0, home);
  ASSERT_EQ(rectangle.size(), 6u);
  EXPECT_TRUE(delta_drawing::pointsNear(rectangle.front(), rectangle[4]));
  EXPECT_TRUE(delta_drawing::pointsNear(rectangle.back(), home));

  const auto triangle = delta_drawing::trianglePath(corners, 20.0, home);
  ASSERT_EQ(triangle.size(), 5u);
  EXPECT_TRUE(delta_drawing::pointsNear(triangle.front(), triangle[3]));
  EXPECT_TRUE(delta_drawing::pointsNear(triangle.back(), home));
}

TEST(ShapePath, CircleUsesExactClosureAndValidatesArguments) {
  const Point center(-100.0, -100.0, -453.0);
  const Point home(0.0, 0.0, -375.0);
  const auto circle = delta_drawing::circlePath(center, 25.0, 20.0, 24, home);

  ASSERT_EQ(circle.size(), 26u);
  EXPECT_DOUBLE_EQ(circle.front().x, circle[24].x);
  EXPECT_DOUBLE_EQ(circle.front().y, circle[24].y);
  EXPECT_DOUBLE_EQ(circle.front().z, circle[24].z);
  EXPECT_TRUE(delta_drawing::pointsNear(circle.back(), home));
  EXPECT_TRUE(delta_drawing::circlePath(center, 0.0, 20.0, 24, home).empty());
  EXPECT_TRUE(delta_drawing::circlePath(center, 25.0, 20.0, 2, home).empty());
}
