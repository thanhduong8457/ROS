/**
 * Unit test for delta_robot kinematics (inverse and trajectory).
 */
#include <gtest/gtest.h>
#include "delta_robot.h"
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
