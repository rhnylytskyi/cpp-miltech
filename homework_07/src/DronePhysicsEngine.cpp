#include "ballistic_app/DronePhysicsEngine.h"
#include "ballistic_app/utils/MathUtils.h"
#include <cmath>

namespace BallisticApp {

namespace {
void processDeceleration(DronePhysicsState& drone, float acceleration, float dt, float& outDeltaPath)
{
  const float prevSpeed = drone.speed;
  drone.speed -= acceleration * dt;
  if (drone.speed <= 0.0f) {
    drone.speed = 0.0f;
    drone.state = DroneState::STOPPED;
  }
  outDeltaPath = (prevSpeed + drone.speed) / 2.0f * dt;
}
}  // namespace

DronePhysicsEngine::DronePhysicsEngine(const DroneConfig& config)
  : m_config(config)
{
}

void DronePhysicsEngine::update(DronePhysicsState& drone, const Coord& firePoint, float dt, float& outDeltaPath) const
{
  const float desiredDirection = std::atan2(firePoint.y - drone.pos.y, firePoint.x - drone.pos.x);
  const float deltaAngle = Math::normalizeAngle(desiredDirection - drone.direction);
  const float acceleration = (m_config.attackSpeed * m_config.attackSpeed) / (2.0f * m_config.accelPath);
  outDeltaPath = 0.0f;

  const bool isTurningRequired = (std::fabs(deltaAngle) > m_config.turnThreshold);

  switch (drone.state) {
    case DroneState::STOPPED:
      if (isTurningRequired) {
        drone.state = DroneState::TURNING;
      }
      else {
        drone.direction = desiredDirection;
        drone.state = DroneState::ACCELERATING;
      }
      break;

    case DroneState::ACCELERATING:
      if (isTurningRequired && drone.speed > 0.01f) {
        drone.state = DroneState::DECELERATING;
        processDeceleration(drone, acceleration, dt, outDeltaPath);
      }
      else {
        if (!isTurningRequired) {
          drone.direction = desiredDirection;
        }
        const float prevSpeed = drone.speed;
        drone.speed += acceleration * dt;
        if (drone.speed >= m_config.attackSpeed) {
          drone.speed = m_config.attackSpeed;
          drone.state = DroneState::MOVING;
        }
        outDeltaPath = (prevSpeed + drone.speed) / 2.0f * dt;
      }
      break;

    case DroneState::DECELERATING:
      processDeceleration(drone, acceleration, dt, outDeltaPath);
      break;

    case DroneState::TURNING: {
      const float da = Math::normalizeAngle(desiredDirection - drone.direction);
      if (std::fabs(da) <= m_config.angularSpeed * dt) {
        drone.direction = desiredDirection;
        drone.state = DroneState::ACCELERATING;
      }
      else {
        drone.direction += (da > 0.0f ? 1.0f : -1.0f) * m_config.angularSpeed * dt;
        drone.direction = Math::normalizeAngle(drone.direction);
      }
      break;
    }

    case DroneState::MOVING:
      if (isTurningRequired) {
        drone.state = DroneState::DECELERATING;
        processDeceleration(drone, acceleration, dt, outDeltaPath);
      }
      else {
        drone.direction = desiredDirection;
        outDeltaPath = drone.speed * dt;
      }
      break;
  }

  drone.pos.x += std::cos(drone.direction) * outDeltaPath;
  drone.pos.y += std::sin(drone.direction) * outDeltaPath;
}

}  // namespace BallisticApp
