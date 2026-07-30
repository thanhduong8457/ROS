/**
 * Unit test for delta_robot kinematics (inverse and trajectory).
 */
#include "cartesian_jog.hpp"
#include "command_validation.hpp"
#include "delta_robot.h"
#include "joint_state_config.hpp"
#include "motion_planner.hpp"
#include "shape_path.hpp"
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>

TEST(DeltaRobot, CheckedInverseAtHomeIsFiniteAndSymmetric) {
  const DeltaRobot robot;
  const auto result =
      robot.inverseChecked(delta_robot_config::kHomeTcpPositionM);

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_TRUE(std::isfinite(result.theta.angle1));
  EXPECT_TRUE(std::isfinite(result.theta.angle2));
  EXPECT_TRUE(std::isfinite(result.theta.angle3));
  EXPECT_NEAR(result.theta.angle1, result.theta.angle2, 1e-6);
  EXPECT_NEAR(result.theta.angle2, result.theta.angle3, 1e-6);
}

TEST(DeltaRobot, CheckedInverseRejectsInvalidPoint) {
  const DeltaRobot robot;
  EXPECT_FALSE(robot.inverseChecked(Point(0.0, 0.0, 0.0)).ok);

  const double infinity = std::numeric_limits<double>::infinity();
  const auto non_finite = robot.inverseChecked(Point(infinity, 0.0, -0.4));
  EXPECT_FALSE(non_finite.ok);
  EXPECT_EQ(non_finite.error, "IK point coordinates must be finite");
}

TEST(DeltaRobot, AllHomeJointPositionsMatchRegressionAndAreFinite) {
  const DeltaRobot robot;
  const Point tcp = delta_robot_config::kHomeTcpPositionM;
  const auto ik = robot.inverseChecked(tcp);
  ASSERT_TRUE(ik.ok) << ik.error;

  DeltaRobot::JointPositions joints{};
  ASSERT_TRUE(robot.createJointStatePositions(tcp, ik.theta, joints));

  const DeltaRobot::JointPositions expected{
      0.000266315,  0.000266533, 0.000266315,  0.9382, 7.38599e-08, 0.9382,
      -1.32324e-11, 0.9382,      -7.38454e-08, 0.0,    0.0,         -0.375,
  };
  for (std::size_t i = 0; i < joints.size(); ++i) {
    EXPECT_TRUE(std::isfinite(joints[i])) << "joint index " << i;
    EXPECT_NEAR(joints[i], expected[i], 1e-4) << "joint index " << i;
  }
}

TEST(DeltaRobot, OffAxisJointMappingIsFiniteAndMapsTcpExactly) {
  const DeltaRobot robot;
  const Point tcp(0.04, -0.025, -0.42);
  const auto ik = robot.inverseChecked(tcp);
  ASSERT_TRUE(ik.ok) << ik.error;

  DeltaRobot::JointPositions joints{};
  ASSERT_TRUE(robot.createJointStatePositions(tcp, ik.theta, joints));
  EXPECT_TRUE(delta_robot_validation::areFinite(joints));
  EXPECT_NE(joints[0], joints[1]);
  EXPECT_NE(joints[1], joints[2]);
  EXPECT_DOUBLE_EQ(joints[9], tcp.x);
  EXPECT_DOUBLE_EQ(joints[10], tcp.y);
  EXPECT_DOUBLE_EQ(joints[11], tcp.z);

  const DeltaRobot::JointPositions expected{
      0.36949160556109412,
      0.14383884308778808,
      0.15305894721082278,
      1.2143858069854947,
      -0.0035497603461579,
      1.1498504181870488,
      -0.08612794908206822,
      1.1522457547132301,
      0.08969146461821546,
      0.04,
      -0.025,
      -0.42,
  };
  for (std::size_t i = 0; i < joints.size(); ++i) {
    EXPECT_NEAR(joints[i], expected[i], 1e-10) << "joint index " << i;
  }
}

TEST(DeltaRobot, JointMappingRejectsNonFiniteInput) {
  const DeltaRobot robot;
  DeltaRobot::JointPositions joints{};
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
      robot.createJointStatePositions(Point(nan, 0.0, -0.4), Theta{}, joints));
  EXPECT_FALSE(robot.createJointStatePositions(Point(0.0, 0.0, -0.4),
                                               Theta{nan, 0.0, 0.0}, joints));
}

TEST(CartesianJog, AcceleratesAlongRequestedAxis) {
  const Point start(0.01, 0.02, -0.4);
  const auto step =
      delta_motion::advanceJog(start, Point(0.0, 1.0, 0.0), 0.0, 0.1, 0.5, 0.1);

  EXPECT_DOUBLE_EQ(step.position_m.x, start.x);
  EXPECT_NEAR(step.position_m.y, 0.0225, 1e-12);
  EXPECT_DOUBLE_EQ(step.position_m.z, start.z);
  EXPECT_NEAR(step.speed_mps, 0.05, 1e-12);
  EXPECT_NEAR(step.acceleration_mps2, 0.5, 1e-12);
}

TEST(CartesianJog, CapsSpeedAndMovesInNegativeDirection) {
  const Point start(0.01, 0.02, -0.4);
  const auto step = delta_motion::advanceJog(start, Point(-1.0, 0.0, 0.0), 0.09,
                                             0.1, 0.5, 0.1);

  EXPECT_NEAR(step.position_m.x, 0.0005, 1e-12);
  EXPECT_DOUBLE_EQ(step.position_m.y, start.y);
  EXPECT_NEAR(step.speed_mps, 0.1, 1e-12);
  EXPECT_NEAR(step.acceleration_mps2, 0.1, 1e-12);
}

TEST(CartesianJog, RecognizesOnlySingleUnitAxisDirections) {
  EXPECT_TRUE(delta_motion::isUnitAxisDirection(Point(1.0, 0.0, 0.0)));
  EXPECT_TRUE(delta_motion::isUnitAxisDirection(Point(0.0, -1.0, 0.0)));
  EXPECT_FALSE(delta_motion::isUnitAxisDirection(Point(0.0, 0.0, 0.0)));
  EXPECT_FALSE(delta_motion::isUnitAxisDirection(Point(1.0, 1.0, 0.0)));
  EXPECT_FALSE(delta_motion::isUnitAxisDirection(Point(1.0, 0.5, 0.0)));
  EXPECT_FALSE(delta_motion::isUnitAxisDirection(Point(0.5, 0.0, 0.0)));
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

TEST(CartesianTrajectory, RejectsInvalidCommonPlanningArguments) {
  const delta_motion::CartesianTrajectoryGenerator generator;
  const delta_motion::MotionLimits valid_limits{1.0, 1.0};
  const Point start(0.0, 0.0, -0.4);
  const Point target(0.1, 0.0, -0.4);

  EXPECT_EQ(generator.planLine(start, start, valid_limits, 0.001).error,
            "Start and target are identical");
  EXPECT_EQ(generator.planLine(start, target, valid_limits, 0.0).error,
            "Sample period must be positive");
  EXPECT_EQ(
      generator
          .planLine(start, target, delta_motion::MotionLimits{0.0, 1.0}, 0.001)
          .error,
      "Velocity and acceleration limits must be positive");
  EXPECT_EQ(generator.planCircle(start, 0.025, false, valid_limits, -0.1).error,
            "Sample period must be positive");
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
  const double nan = std::numeric_limits<double>::quiet_NaN();

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
  EXPECT_TRUE(delta_robot_validation::pointsNear(
      Point(0.0, 0.0, 0.0), Point(0.0001, 0.0, 0.0), 0.001));
  EXPECT_FALSE(delta_robot_validation::pointsNear(
      Point(0.0, 0.0, 0.0), Point(0.0001, 0.0, 0.0), -1.0));
  EXPECT_FALSE(
      delta_robot_validation::areFinite(std::array<double, 2>{0.0, nan}));
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

TEST(ShapePath, RejectsNonFinitePointsAndOffsets) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::array<Point, 4> corners = {
      Point{0.0, 10.0, -453.0},
      Point{-10.0, 0.0, -453.0},
      Point{0.0, -10.0, -453.0},
      Point{10.0, 0.0, -453.0},
  };
  const Point home(0.0, 0.0, -375.0);

  corners[1].x = nan;
  EXPECT_TRUE(delta_drawing::rectanglePath(corners, 20.0, home).empty());
  EXPECT_TRUE(delta_drawing::trianglePath(corners, 20.0, home).empty());
  EXPECT_TRUE(
      delta_drawing::circlePath(Point(nan, 0.0, -453.0), 25.0, 20.0, 24, home)
          .empty());
  EXPECT_TRUE(
      delta_drawing::circlePath(Point(0.0, 0.0, -453.0), 25.0, nan, 24, home)
          .empty());
  EXPECT_TRUE(
      delta_drawing::circlePath(Point(0.0, 0.0, -453.0), 25.0, -1.0, 24, home)
          .empty());
  EXPECT_TRUE(
      delta_drawing::circlePath(Point(0.0, 0.0, -453.0), 25.0, 20.0,
                                delta_drawing::kMaxShapeSegmentCount + 1, home)
          .empty());
}
