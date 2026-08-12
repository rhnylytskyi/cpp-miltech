#pragma once

#include "BallisticApp/sys/UdpSocket.h"
#include "drone_link.h"
#include <string>
#include <cstdint>
#include <chrono>  // Added for steady_clock

namespace BallisticApp::sys {

class MavlinkManager {
public:
  MavlinkManager(const std::string& host, uint16_t port, uint8_t sysId = 1);
  ~MavlinkManager() noexcept = default;

  MavlinkManager(const MavlinkManager&) = delete;
  MavlinkManager& operator=(const MavlinkManager&) = delete;

  void processIncoming() noexcept;
  void processDropFsm(float timeScale = 1.0f) noexcept;
  void handleTelemetry(const dlink::Telemetry& tel, float timeScale = 1.0f) noexcept;
  void triggerDrop(float x, float y, float z) noexcept;

  [[nodiscard]] bool isEnabled() const noexcept { return m_enabled; }
  [[nodiscard]] bool isDropAckPending() const noexcept;

private:
  void sendHeartbeat() noexcept;
  void sendTelemetry(const dlink::Telemetry& tel) noexcept;
  void sendDropCommand(int attempt) noexcept;
  void localToGeo(float x, float y, double& latDeg, double& lonDeg) const noexcept;

  enum class DropState { IDLE, SEND_AND_WAIT, ACK_RECEIVED, FAILED };

  UdpSocket m_socket;
  bool m_enabled{false};
  uint8_t m_sysId{1};

  // Real-time tracking clock points
  std::chrono::steady_clock::time_point m_lastHeartbeatTime;
  std::chrono::steady_clock::time_point m_lastTelemetryTime;
  std::chrono::steady_clock::time_point m_lastDropSendTime;

  DropState m_dropState{DropState::IDLE};
  int m_dropAttempts{0};
  double m_dropLatDeg{0.0};
  double m_dropLonDeg{0.0};
  float m_dropAltitudeM{0.0f};
};

}  // namespace BallisticApp::sys
