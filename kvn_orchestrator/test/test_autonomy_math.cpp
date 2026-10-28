// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.
//
// Assisted-teleop carrot and battery return-to-home logic (no ROS).

#include <gtest/gtest.h>

#include <cmath>

#include "kvn_orchestrator/assisted_teleop.hpp"
#include "kvn_orchestrator/battery_model.hpp"

using kvn_orchestrator::BatteryReturnParams;
using kvn_orchestrator::CarrotParams;
using kvn_orchestrator::Pose2D;
using kvn_orchestrator::ReturnReason;

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

TEST(CarrotGoal, FullForwardStickPlacesTheGoalAheadAtTheLookahead)
{
  CarrotParams params;
  params.lookahead_m = 2.0;
  params.deadzone = 0.0;
  const auto goal = kvn_orchestrator::carrot_goal(Pose2D{1.0, 1.0, kPi / 2}, 1.0, 0.0, params);
  EXPECT_NEAR(goal.x, 1.0, 1e-9);
  EXPECT_NEAR(goal.y, 3.0, 1e-9);
  EXPECT_NEAR(goal.yaw, kPi / 2, 1e-9);
}

TEST(CarrotGoal, SidewaysStickTurnsTheGoalHeadingCounterClockwiseForLeft)
{
  CarrotParams params;
  params.lookahead_m = 1.0;
  params.max_heading_rad = kPi / 2;
  params.deadzone = 0.0;
  const auto goal = kvn_orchestrator::carrot_goal(Pose2D{}, 1.0, 1.0, params);
  EXPECT_NEAR(goal.x, 0.0, 1e-9);
  EXPECT_NEAR(goal.y, 1.0, 1e-9);
  EXPECT_NEAR(goal.yaw, kPi / 2, 1e-9);
}

TEST(CarrotGoal, ShortOrBackwardStickTurnsOnTheSpot)
{
  CarrotParams params;
  params.lookahead_m = 1.0;
  params.min_distance_m = 0.3;
  params.max_heading_rad = 1.0;
  params.deadzone = 0.0;
  const Pose2D robot{2.0, -1.0, 0.5};
  for (const double linear : {0.1, -1.0}) {
    const auto goal = kvn_orchestrator::carrot_goal(robot, linear, -0.5, params);
    EXPECT_DOUBLE_EQ(goal.x, robot.x);
    EXPECT_DOUBLE_EQ(goal.y, robot.y);
    EXPECT_NEAR(goal.yaw, 0.0, 1e-9);
  }
}

TEST(CarrotGoal, DeadzoneIsRemovedAndTheRestRescaled)
{
  EXPECT_DOUBLE_EQ(kvn_orchestrator::apply_deadzone(0.05, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(kvn_orchestrator::apply_deadzone(1.0, 0.1), 1.0);
  EXPECT_NEAR(kvn_orchestrator::apply_deadzone(-0.55, 0.1), -0.5, 1e-9);
  EXPECT_DOUBLE_EQ(kvn_orchestrator::apply_deadzone(3.0, 0.0), 1.0);
}

TEST(CarrotGoal, HeadingWrapsAroundPi)
{
  CarrotParams params;
  params.deadzone = 0.0;
  params.max_heading_rad = 1.0;
  const auto goal = kvn_orchestrator::carrot_goal(Pose2D{0, 0, 3.0}, 0.0, 1.0, params);
  EXPECT_NEAR(goal.yaw, 4.0 - 2 * kPi, 1e-9);
}

TEST(CarrotGoal, GoalMovedUsesDistanceOrAngle)
{
  const Pose2D a{0.0, 0.0, 3.1};
  EXPECT_FALSE(kvn_orchestrator::goal_moved(a, Pose2D{0.1, 0.0, -3.1}, 0.2, 0.2));
  EXPECT_TRUE(kvn_orchestrator::goal_moved(a, Pose2D{0.3, 0.0, 3.1}, 0.2, 0.2));
  EXPECT_TRUE(kvn_orchestrator::goal_moved(a, Pose2D{0.0, 0.0, 2.8}, 0.2, 0.2));
}

TEST(BatteryModel, StateOfChargeInterpolatesTheCellCurve)
{
  const std::vector<double> curve{3.0, 3.5, 4.0};
  EXPECT_DOUBLE_EQ(kvn_orchestrator::state_of_charge_from_voltage(11.0, 4, curve), 0.0);
  EXPECT_DOUBLE_EQ(kvn_orchestrator::state_of_charge_from_voltage(14.0, 4, curve), 0.5);
  EXPECT_DOUBLE_EQ(kvn_orchestrator::state_of_charge_from_voltage(15.0, 4, curve), 0.75);
  EXPECT_DOUBLE_EQ(kvn_orchestrator::state_of_charge_from_voltage(17.0, 4, curve), 1.0);
  EXPECT_DOUBLE_EQ(kvn_orchestrator::state_of_charge_from_voltage(NAN, 4, curve), 0.0);
}

TEST(BatteryModel, FixedThresholdTriggersWhateverTheDistance)
{
  BatteryReturnParams params;
  const auto check = kvn_orchestrator::check_battery_return(12.4, 0.9, 0.0, params);
  EXPECT_EQ(check.reason, ReturnReason::kCriticalVoltage);
  EXPECT_EQ(
    kvn_orchestrator::check_battery_return(NAN, NAN, 0.0, params).reason,
    ReturnReason::kCriticalVoltage);
}

TEST(BatteryModel, EnergyCheckComparesChargeWithTheTripHome)
{
  BatteryReturnParams params;
  params.capacity_wh = 40.0;
  params.wh_per_m = 0.02;
  params.path_factor = 1.5;
  params.reserve_wh = 4.0;
  // 20 % of 40 Wh = 8 Wh. Home 100 m away needs 100 * 1.5 * 0.02 + 4 = 7 Wh: keep going.
  auto check = kvn_orchestrator::check_battery_return(15.0, 0.2, 100.0, params);
  EXPECT_FALSE(check.required());
  EXPECT_NEAR(check.remaining_wh, 8.0, 1e-9);
  EXPECT_NEAR(check.required_wh, 7.0, 1e-9);
  // 150 m away needs 8.5 Wh: go home now.
  check = kvn_orchestrator::check_battery_return(15.0, 0.2, 150.0, params);
  EXPECT_EQ(check.reason, ReturnReason::kEnergy);
}

TEST(BatteryModel, VoltageCurveIsUsedWithoutAPercentage)
{
  BatteryReturnParams params;
  params.cells = 4;
  params.cell_voltage_curve = {3.0, 4.0};
  params.capacity_wh = 10.0;
  params.reserve_wh = 3.0;
  params.wh_per_m = 0.0;
  // 13.0 V = 3.25 V/cell = 25 % = 2.5 Wh, below the 3 Wh reserve.
  EXPECT_EQ(
    kvn_orchestrator::check_battery_return(13.0, NAN, 0.0, params).reason, ReturnReason::kEnergy);
  EXPECT_FALSE(kvn_orchestrator::check_battery_return(14.0, NAN, 0.0, params).required());
}
