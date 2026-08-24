#include "antidrone_turret/turret_decision.hpp"
#include <gtest/gtest.h>

namespace {
using antidrone_turret::AxisDirection;
using antidrone_turret::TargetState;
using antidrone_turret::TriggerDecision;
using antidrone_turret::TurretAction;
using antidrone_turret::TurretDecisionConfig;
using antidrone_turret::TurretDecisionInput;

TEST(TurretDecisionTest, LowConfidenceGoesIdleAndSkipsTrigger)
{
  TurretDecisionInput inputData;
  inputData.visible = true;
  inputData.x = 400.0f;
  inputData.y = 200.0f;
  inputData.distance_m = 10.0f;
  inputData.confidence = 0.5f;
  inputData.actuator_ready = true;

  const auto calculatedDecision = antidrone_turret::evaluate(inputData, TurretDecisionConfig{});

  EXPECT_EQ(calculatedDecision.target_state, TargetState::kLowConfidence);
  EXPECT_EQ(calculatedDecision.action, TurretAction::kIdle);
  EXPECT_EQ(calculatedDecision.trigger_state, TriggerDecision::kSkip);
  EXPECT_FALSE(calculatedDecision.has_gimbal_command);
  EXPECT_FALSE(calculatedDecision.has_servo_command);
}

TEST(TurretDecisionTest, NotVisibleGoesNoneAndIdle)
{
  TurretDecisionInput inputData;
  inputData.visible = false;

  const auto calculatedDecision = antidrone_turret::evaluate(inputData, TurretDecisionConfig{});

  EXPECT_EQ(calculatedDecision.target_state, TargetState::kNone);
  EXPECT_EQ(calculatedDecision.action, TurretAction::kIdle);
  EXPECT_EQ(calculatedDecision.trigger_state, TriggerDecision::kSkip);
}

TEST(TurretDecisionTest, ServoCommandPointsRightWhenTargetIsRightOfCenter)
{
  const auto activeCommand = antidrone_turret::compute_servo_command(420.0f);

  EXPECT_EQ(activeCommand.direction, AxisDirection::kPositive);
  EXPECT_FLOAT_EQ(activeCommand.target_x, 420.0f);
  EXPECT_GT(activeCommand.error_x, 0.0f);
  EXPECT_FLOAT_EQ(activeCommand.error_x, 100.0f);
}

TEST(TurretDecisionTest, ServoCommandPointsLeftWhenTargetIsLeftOfCenter)
{
  const auto activeCommand = antidrone_turret::compute_servo_command(220.0f);

  EXPECT_EQ(activeCommand.direction, AxisDirection::kNegative);
  EXPECT_LT(activeCommand.error_x, 0.0f);
}

TEST(TurretDecisionTest, ServoCommandCentersWhenTargetIsOnCenter)
{
  const auto activeCommand = antidrone_turret::compute_servo_command(320.0f);

  EXPECT_EQ(activeCommand.direction, AxisDirection::kCenter);
  EXPECT_FLOAT_EQ(activeCommand.error_x, 0.0f);
}

TEST(TurretDecisionTest, GimbalCommandPointsUpWhenTargetIsAboveCenter)
{
  const auto activeCommand = antidrone_turret::compute_gimbal_command(180.0f);

  EXPECT_EQ(activeCommand.direction, AxisDirection::kPositive);
  EXPECT_FLOAT_EQ(activeCommand.target_y, 180.0f);
  EXPECT_GT(activeCommand.error_y, 0.0f);
  EXPECT_FLOAT_EQ(activeCommand.error_y, 60.0f);
}

TEST(TurretDecisionTest, GimbalCommandPointsDownWhenTargetIsBelowCenter)
{
  const auto activeCommand = antidrone_turret::compute_gimbal_command(300.0f);

  EXPECT_EQ(activeCommand.direction, AxisDirection::kNegative);
  EXPECT_LT(activeCommand.error_y, 0.0f);
}

TEST(TurretDecisionTest, CloseTargetWithReadyActuatorRequestsTrigger)
{
  const auto calculatedDecision = antidrone_turret::decide_trigger(25.0f, 30.0f, true);
  EXPECT_EQ(calculatedDecision, TriggerDecision::kRequested);
}

TEST(TurretDecisionTest, CloseTargetWithReloadingActuatorDoesNotRequestTrigger)
{
  const auto calculatedDecision = antidrone_turret::decide_trigger(25.0f, 30.0f, false);
  EXPECT_EQ(calculatedDecision, TriggerDecision::kReloading);
}

TEST(TurretDecisionTest, FarTargetSkipsTriggerRegardlessOfActuatorState)
{
  EXPECT_EQ(antidrone_turret::decide_trigger(45.0f, 30.0f, true), TriggerDecision::kSkip);
  EXPECT_EQ(antidrone_turret::decide_trigger(45.0f, 30.0f, false), TriggerDecision::kSkip);
}

TEST(TurretDecisionTest, FarLockedTargetTracksButSkipsTrigger)
{
  TurretDecisionInput inputData;
  inputData.visible = true;
  inputData.x = 420.0f;
  inputData.y = 180.0f;
  inputData.distance_m = 70.0f;
  inputData.confidence = 0.90f;
  inputData.actuator_ready = true;

  const auto calculatedDecision = antidrone_turret::evaluate(inputData, TurretDecisionConfig{});

  EXPECT_EQ(calculatedDecision.target_state, TargetState::kLocked);
  EXPECT_EQ(calculatedDecision.action, TurretAction::kTrack);
  EXPECT_EQ(calculatedDecision.trigger_state, TriggerDecision::kSkip);
  ASSERT_TRUE(calculatedDecision.has_gimbal_command);
  ASSERT_TRUE(calculatedDecision.has_servo_command);
  EXPECT_EQ(calculatedDecision.gimbal_command.direction, AxisDirection::kPositive);
  EXPECT_EQ(calculatedDecision.servo_command.direction, AxisDirection::kPositive);
}

TEST(TurretDecisionTest, CloseLockedTargetWithReadyActuatorRequestsTriggerThroughEvaluate)
{
  TurretDecisionInput inputData;
  inputData.visible = true;
  inputData.x = 320.0f;
  inputData.y = 240.0f;
  inputData.distance_m = 19.0f;
  inputData.confidence = 0.95f;
  inputData.actuator_ready = true;

  const auto calculatedDecision = antidrone_turret::evaluate(inputData, TurretDecisionConfig{});

  EXPECT_EQ(calculatedDecision.target_state, TargetState::kLocked);
  EXPECT_EQ(calculatedDecision.action, TurretAction::kTrack);
  EXPECT_EQ(calculatedDecision.trigger_state, TriggerDecision::kRequested);
}
}  // namespace
