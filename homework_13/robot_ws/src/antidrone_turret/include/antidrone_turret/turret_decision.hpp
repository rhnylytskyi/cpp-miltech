#pragma once

#include <cstdint>

namespace antidrone_turret {
// Target frame dimensions (640x480). Resolution center offsets.
constexpr float frameCenterX = 320.0f;
constexpr float frameCenterY = 240.0f;

enum class TargetState : std::uint8_t {
  kNone = 0,
  kLowConfidence = 1,
  kLocked = 2,
};

enum class TurretAction : std::uint8_t {
  kIdle = 0,
  kTrack = 1,
};

enum class TriggerDecision : std::uint8_t {
  kSkip = 0,
  kRequested = 1,
  kReloading = 2,
};

enum class AxisDirection : std::int8_t {
  kNegative = -1,
  kCenter = 0,
  kPositive = 1,
};

struct ServoCommandDecision {
  AxisDirection direction{AxisDirection::kCenter};
  float target_x{0.0f};
  float error_x{0.0f};
};

struct GimbalCommandDecision {
  AxisDirection direction{AxisDirection::kCenter};
  float target_y{0.0f};
  float error_y{0.0f};
};

struct TurretDecisionInput {
  bool visible{false};
  float x{0.0f};
  float y{0.0f};
  float distance_m{0.0f};
  float confidence{0.0f};
  bool actuator_ready{true};
};

struct TurretDecisionConfig {
  float confidence_threshold{0.80f};
  float max_distance_m{30.0f};
};

struct TurretDecision {
  TargetState target_state{TargetState::kNone};
  TurretAction action{TurretAction::kIdle};
  TriggerDecision trigger_state{TriggerDecision::kSkip};
  float confidence{0.0f};
  float distance_m{0.0f};

  bool has_gimbal_command{false};
  GimbalCommandDecision gimbal_command{};

  bool has_servo_command{false};
  ServoCommandDecision servo_command{};
};

[[nodiscard]] inline TargetState evaluate_target_state(const bool isVisible, const float targetConfidence, const float minThreshold)
{
  if (!isVisible)
    return TargetState::kNone;
  if (targetConfidence < minThreshold)
    return TargetState::kLowConfidence;
  return TargetState::kLocked;
}

[[nodiscard]] inline ServoCommandDecision compute_servo_command(const float targetPosX)
{
  ServoCommandDecision commandOutput;
  commandOutput.target_x = targetPosX;
  commandOutput.error_x = targetPosX - frameCenterX;

  commandOutput.direction = (commandOutput.error_x > 0.0f)   ? AxisDirection::kPositive
                            : (commandOutput.error_x < 0.0f) ? AxisDirection::kNegative
                                                             : AxisDirection::kCenter;

  return commandOutput;
}

[[nodiscard]] inline GimbalCommandDecision compute_gimbal_command(const float targetPosY)
{
  GimbalCommandDecision commandOutput;
  commandOutput.target_y = targetPosY;
  commandOutput.error_y = frameCenterY - targetPosY;

  commandOutput.direction = (commandOutput.error_y > 0.0f)   ? AxisDirection::kPositive
                            : (commandOutput.error_y < 0.0f) ? AxisDirection::kNegative
                                                             : AxisDirection::kCenter;

  return commandOutput;
}

[[nodiscard]] inline TriggerDecision decide_trigger(const float currentRange, const float maxRange, const bool isActuatorReady)
{
  if (currentRange > maxRange)
    return TriggerDecision::kSkip;
  return isActuatorReady ? TriggerDecision::kRequested : TriggerDecision::kReloading;
}

[[nodiscard]] inline TurretDecision evaluate(const TurretDecisionInput &inputData, const TurretDecisionConfig &currentConfig)
{
  TurretDecision finalDecision;
  finalDecision.confidence = inputData.confidence;
  finalDecision.distance_m = inputData.distance_m;
  finalDecision.target_state = evaluate_target_state(inputData.visible, inputData.confidence, currentConfig.confidence_threshold);

  if (finalDecision.target_state != TargetState::kLocked) {
    finalDecision.action = TurretAction::kIdle;
    finalDecision.trigger_state = TriggerDecision::kSkip;
    return finalDecision;
  }

  finalDecision.action = TurretAction::kTrack;

  finalDecision.has_gimbal_command = true;
  finalDecision.gimbal_command = compute_gimbal_command(inputData.y);

  finalDecision.has_servo_command = true;
  finalDecision.servo_command = compute_servo_command(inputData.x);

  finalDecision.trigger_state = decide_trigger(inputData.distance_m, currentConfig.max_distance_m, inputData.actuator_ready);

  return finalDecision;
}
}  // namespace antidrone_turret
