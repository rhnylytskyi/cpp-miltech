#include "BallisticApp/sys/MavlinkManager.h"
#include "BallisticApp/utils/Logger.h"
#include "BallisticApp/utils/MathUtils.h"
#include <algorithm>
#include <cmath>
#include <common/mavlink.h>

namespace BallisticApp::sys {

namespace {
constexpr double kLat0 = 50.4501;
constexpr double kLon0 = 30.5234;
constexpr double kMetersPerDegLat = 111320.0;
constexpr uint8_t kCompId = MAV_COMP_ID_AUTOPILOT1;
constexpr int kMaxDropAttempts = 5;
}  // namespace

MavlinkManager::MavlinkManager(const std::string& host, uint16_t port, uint8_t sysId)
  : m_sysId(sysId)
{
  try {
    m_socket.open(host, port);
    m_enabled = true;

    // Initialize real-time clock anchors to ancient past to force immediate first tick execution
    auto ancientPast = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    m_lastHeartbeatTime = ancientPast;
    m_lastTelemetryTime = ancientPast;
    m_lastDropSendTime = ancientPast;

    APP_LOG_MOD("Network", "MAVLink engine online bound to {}:{}", host, port);
  }
  catch (const std::exception& e) {
    APP_LOG_MOD("Network", "MAVLink initialization failed: {}", e.what());
    m_enabled = false;
  }
}

bool MavlinkManager::isDropAckPending() const noexcept
{
  return m_dropState == DropState::SEND_AND_WAIT;
}

void MavlinkManager::localToGeo(float x, float y, double& latDeg, double& lonDeg) const noexcept
{
  latDeg = kLat0 + static_cast<double>(y) / kMetersPerDegLat;
  lonDeg = kLon0 + static_cast<double>(x) / (kMetersPerDegLat * std::cos(kLat0 * M_PI / 180.0));
}

void MavlinkManager::processIncoming() noexcept
{
  if (!m_enabled)
    return;

  uint8_t buffer[512];
  int receivedBytes = m_socket.recv(buffer, sizeof(buffer));
  if (receivedBytes <= 0)
    return;

  mavlink_status_t status;
  mavlink_message_t msg;
  for (int i = 0; i < receivedBytes; ++i) {
    if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status) != 0) {
      if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(&msg, &ack);

        if (m_dropState == DropState::SEND_AND_WAIT && ack.command == MAV_CMD_USER_1) {
          m_dropState = DropState::ACK_RECEIVED;
          APP_LOG_MOD("Network", "MAVLink DROP: ACK received (result={})", static_cast<int>(ack.result));
        }
      }
    }
  }
}

void MavlinkManager::processDropFsm(float timeScale) noexcept
{
  if (!m_enabled || m_dropState != DropState::SEND_AND_WAIT)
    return;

  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastDropSendTime).count();

  // Dynamic retry timeout adapted to compressed physics speed
  int dynamicDropTimeoutMs = static_cast<int>(1000.0f / std::max(0.01f, timeScale));

  if (elapsed >= dynamicDropTimeoutMs) {
    if (m_dropAttempts < kMaxDropAttempts) {
      sendDropCommand(m_dropAttempts);
      m_dropAttempts++;
      m_lastDropSendTime = now;
    }
    else {
      m_dropState = DropState::FAILED;
      APP_LOG_MOD("Network", "MAVLink DROP: ACK timeout after {} attempts", kMaxDropAttempts);
    }
  }
}

void MavlinkManager::handleTelemetry(const dlink::Telemetry& tel, float timeScale) noexcept
{
  if (!m_enabled)
    return;

  auto now = std::chrono::steady_clock::now();
  float scale = std::max(0.01f, timeScale);

  // Both heartbeat and position must scale according to simulation time compression
  int targetHeartbeatIntervalMs = static_cast<int>(1000.0f / scale);
  int targetTelemetryIntervalMs = static_cast<int>(500.0f / scale);

  // Heartbeat processing window
  auto heartbeatElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastHeartbeatTime).count();
  if (heartbeatElapsed >= targetHeartbeatIntervalMs) {
    sendHeartbeat();
    m_lastHeartbeatTime = now;
  }

  // Position telemetry streaming window
  auto telemetryElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastTelemetryTime).count();
  if (telemetryElapsed >= targetTelemetryIntervalMs) {
    sendTelemetry(tel);
    m_lastTelemetryTime = now;
  }
}

void MavlinkManager::triggerDrop(float x, float y, float z) noexcept
{
  if (!m_enabled)
    return;

  localToGeo(x, y, m_dropLatDeg, m_dropLonDeg);
  m_dropAltitudeM = z;
  m_dropState = DropState::SEND_AND_WAIT;
  m_dropAttempts = 0;

  // Forces immediate execution on the very next FSM processing tick loop
  m_lastDropSendTime = std::chrono::steady_clock::now() - std::chrono::seconds(2);
}

void MavlinkManager::sendHeartbeat() noexcept
{
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(m_sysId,
                             kCompId,
                             &msg,
                             MAV_TYPE_QUADROTOR,
                             MAV_AUTOPILOT_GENERIC,
                             MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                             0,
                             MAV_STATE_ACTIVE);

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  m_socket.send(buf, len);
}

void MavlinkManager::sendTelemetry(const dlink::Telemetry& tel) noexcept
{
  double latDeg = 0.0, lonDeg = 0.0;
  localToGeo(tel.x, tel.y, latDeg, lonDeg);

  int32_t lat = static_cast<int32_t>(std::lround(latDeg * 1e7));
  int32_t lon = static_cast<int32_t>(std::lround(lonDeg * 1e7));
  int32_t altMm = static_cast<int32_t>(std::lround(tel.z * 1000.0f));

  auto toCm = [](float v) -> int16_t {
    float cm = v * 100.0f;
    cm = std::clamp(cm, -32768.0f, 32767.0f);
    return static_cast<int16_t>(std::lround(cm));
  };

  int16_t vx = toCm(tel.vx);
  int16_t vy = toCm(tel.vy);

  float headingDeg = 90.0f - tel.dir * 180.0f / static_cast<float>(M_PI);
  headingDeg = std::fmod(headingDeg, 360.0f);
  if (headingDeg < 0.0f)
    headingDeg += 360.0f;
  uint16_t hdg = static_cast<uint16_t>(std::lround(headingDeg * 100.0f)) % 36000;

  uint32_t bootMs = tel.t_ms;

  mavlink_message_t msg;
  mavlink_msg_global_position_int_pack(m_sysId, kCompId, &msg, bootMs, lat, lon, altMm, altMm, vx, vy, 0, hdg);

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  m_socket.send(buf, len);

  float yaw = Math::normalizeAngle(static_cast<float>(M_PI) / 2.0f - tel.dir);
  mavlink_msg_attitude_pack(m_sysId, kCompId, &msg, bootMs, 0.0f, 0.0f, yaw, 0.0f, 0.0f, 0.0f);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  m_socket.send(buf, len);
}

void MavlinkManager::sendDropCommand(int attempt) noexcept
{
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(m_sysId,
                                kCompId,
                                &msg,
                                0,
                                0,
                                MAV_CMD_USER_1,
                                static_cast<uint8_t>(attempt),
                                0.0f,
                                0.0f,
                                0.0f,
                                0.0f,
                                static_cast<float>(m_dropLatDeg),
                                static_cast<float>(m_dropLonDeg),
                                m_dropAltitudeM);

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  m_socket.send(buf, len);

  APP_LOG_MOD("Network", "MAVLink DROP: COMMAND_LONG sent. Attempt {}/{}", attempt + 1, kMaxDropAttempts);
}

}  // namespace BallisticApp::sys
