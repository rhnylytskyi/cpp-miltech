#include "ballistic_app/DronePhysicsEngine.h"
#include "ballistic_app/utils/MathUtils.h"
#include <cmath>

namespace BallisticApp {
DronePhysicsEngine::DronePhysicsEngine(const DroneConfig& config)
  : m_config(config)
{
}

void DronePhysicsEngine::update(DronePhysicsState& drone, const Coord& firePoint, float dt, float& outDeltaPath) const
{
  float desiredDirection = std::atan2(firePoint.y - drone.pos.y, firePoint.x - drone.pos.x);
  float deltaAngle = Math::normalizeAngle(desiredDirection - drone.direction);
  float acceleration = (m_config.attackSpeed * m_config.attackSpeed) / (2.0f * m_config.accelPath);
  outDeltaPath = 0.0f;

  switch (drone.state) {
    case DroneState::STOPPED:
      if (std::fabs(deltaAngle) > m_config.turnThreshold)
        drone.state = DroneState::TURNING;
      else {
        drone.direction = desiredDirection;
        drone.state = DroneState::ACCELERATING;
      }
      break;
    case DroneState::ACCELERATING:
      if (std::fabs(deltaAngle) > m_config.turnThreshold && drone.speed > 0.01f) {
        drone.state = DroneState::DECELERATING;
        float prevSpeed = drone.speed;
        drone.speed -= acceleration * dt;
        if (drone.speed <= 0) {
          drone.speed = 0;
          drone.state = DroneState::STOPPED;
        }
        outDeltaPath = (prevSpeed + drone.speed) / 2.0f * dt;
      }
      else {
        if (std::fabs(deltaAngle) <= m_config.turnThreshold)
          drone.direction = desiredDirection;
        float prevSpeed = drone.speed;
        drone.speed += acceleration * dt;
        if (drone.speed >= m_config.attackSpeed) {
          drone.speed = m_config.attackSpeed;
          drone.state = DroneState::MOVING;
        }
        outDeltaPath = (prevSpeed + drone.speed) / 2.0f * dt;
      }
      break;
    case DroneState::DECELERATING: {
      float prevSpeed = drone.speed;
      drone.speed -= acceleration * dt;
      if (drone.speed <= 0) {
        drone.speed = 0;
        drone.state = DroneState::STOPPED;
      }
      outDeltaPath = (prevSpeed + drone.speed) / 2.0f * dt;
      break;
    }
    case DroneState::TURNING: {
      float da = Math::normalizeAngle(desiredDirection - drone.direction);
      if (std::fabs(da) <= m_config.angularSpeed * dt) {
        drone.direction = desiredDirection;
        drone.state = DroneState::ACCELERATING;
      }
      else {
        drone.direction += (da > 0 ? 1.0f : -1.0f) * m_config.angularSpeed * dt;
        drone.direction = Math::normalizeAngle(drone.direction);
      }
      break;
    }
    case DroneState::MOVING:
      if (std::fabs(deltaAngle) > m_config.turnThreshold) {
        drone.state = DroneState::DECELERATING;
        float prevSpeed = drone.speed;
        drone.speed -= acceleration * dt;
        if (drone.speed <= 0) {
          drone.speed = 0;
          drone.state = DroneState::STOPPED;
        }
        outDeltaPath = (prevSpeed + drone.speed) / 2.0f * dt;
      }
      else {
        if (std::fabs(deltaAngle) <= m_config.turnThreshold)
          drone.direction = desiredDirection;
        outDeltaPath = drone.speed * dt;
      }
      break;
  }
  drone.pos.x += std::cos(drone.direction) * outDeltaPath;
  drone.pos.y += std::sin(drone.direction) * outDeltaPath;
}
}  // namespace BallisticApp
