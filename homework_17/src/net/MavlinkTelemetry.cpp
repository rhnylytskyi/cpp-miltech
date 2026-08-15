#include "BallisticApp/utils/Logger.h"
#define _USE_MATH_DEFINES
#include "BallisticApp/net/MavlinkTelemetry.h"
#include <common/mavlink.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>

namespace BallisticApp {

constexpr double kLat0 = 50.4501;
constexpr double kLon0 = 30.5234;
constexpr double kMetersPerDegLat = 111320.0;
constexpr uint8_t kCompId = MAV_COMP_ID_AUTOPILOT1;
constexpr int kDropMaxAttempts = 5;
constexpr int kDropAckTimeoutMs = 1000;

MavlinkTelemetry::MavlinkTelemetry(std::string host, uint16_t port, uint8_t sysId)
  : m_host(std::move(host))
  , m_port(port)
  , m_sysId(sysId)
{
}

MavlinkTelemetry::~MavlinkTelemetry()
{
  stop();
}

void MavlinkTelemetry::localToGeo(float x, float y, double& latDeg, double& lonDeg)
{
  latDeg = kLat0 + static_cast<double>(y) / kMetersPerDegLat;
  lonDeg = kLon0 + static_cast<double>(x) / (kMetersPerDegLat * std::cos(kLat0 * M_PI / 180.0));
}

void MavlinkTelemetry::start()
{
  if (m_running.load())
    return;
  m_sock.open(m_host, m_port);
  m_running.store(true);
  m_heartbeatThread = std::thread([this] { heartbeatLoop(); });
  m_commandThread = std::thread([this] { commandLoop(); });
  m_rxThread = std::thread([this] { rxLoop(); });
}

void MavlinkTelemetry::stop()
{
  if (!m_running.load())
    return;
  m_running.store(false);

  m_sock.close();
  m_dropCv.notify_all();
  m_ackCv.notify_all();

  if (m_heartbeatThread.joinable())
    m_heartbeatThread.join();
  if (m_commandThread.joinable())
    m_commandThread.join();

  if (m_rxThread.joinable()) {
    m_rxThread.detach();
  }
}

void MavlinkTelemetry::sendBuffer(const uint8_t* buf, uint16_t len)
{
  m_sock.send(buf, len);
}

void MavlinkTelemetry::feedTelemetry(Coord pos, Coord speed, float direction, float altitude, uint32_t simTimeMs)
{
  if (!m_running.load())
    return;

  double latDeg = 0.0, lonDeg = 0.0;
  localToGeo(pos.x, pos.y, latDeg, lonDeg);

  int32_t lat = static_cast<int32_t>(std::lround(latDeg * 1e7));
  int32_t lon = static_cast<int32_t>(std::lround(lonDeg * 1e7));
  int32_t altMm = static_cast<int32_t>(std::lround(altitude * 1000.0f));

  auto toCm = [](float v) -> int16_t {
    float cm = v * 100.0f;
    cm = std::max(-32768.0f, std::min(32767.0f, cm));
    return static_cast<int16_t>(std::lround(cm));
  };

  int16_t vx = toCm(speed.x);
  int16_t vy = toCm(speed.y);

  float headingDeg = 90.0f - direction * 180.0f / static_cast<float>(M_PI);
  headingDeg = std::fmod(headingDeg, 360.0f);
  if (headingDeg < 0.0f)
    headingDeg += 360.0f;
  uint16_t hdg = static_cast<uint16_t>(std::lround(headingDeg * 100.0f)) % 36000;

  mavlink_message_t msg;
  mavlink_msg_global_position_int_pack(m_sysId, kCompId, &msg, simTimeMs, lat, lon, altMm, altMm, vx, vy, 0, hdg);

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  sendBuffer(buf, len);

  float yaw = static_cast<float>(M_PI) / 2.0f - direction;
  mavlink_msg_attitude_pack(m_sysId, kCompId, &msg, simTimeMs, 0.0f, 0.0f, yaw, 0.0f, 0.0f, 0.0f);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  sendBuffer(buf, len);
}

void MavlinkTelemetry::notifyDrop(Coord dropPointLocal, float altitude)
{
  m_dropActive.store(true);
  {
    std::lock_guard<std::mutex> lock(m_dropMtx);
    m_dropPoint = dropPointLocal;
    m_dropAltitude_ = altitude;
    m_dropPending = true;
  }
  m_dropCv.notify_all();
}

void MavlinkTelemetry::waitDropSettled()
{
  std::unique_lock<std::mutex> lock(m_dropMtx);
  m_dropDoneCv.wait(lock, [this] { return !m_dropActive.load() || !m_running.load(); });
}

void MavlinkTelemetry::heartbeatLoop()
{
  while (m_running.load()) {
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
    sendBuffer(buf, len);

    for (int i = 0; i < 10 && m_running.load(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
}

void MavlinkTelemetry::commandLoop()
{
  while (m_running.load()) {
    Coord dp{0.0f, 0.0f};
    float alt = 0.0f;
    {
      std::unique_lock<std::mutex> lock(m_dropMtx);
      m_dropCv.wait(lock, [this] { return m_dropPending || !m_running.load(); });
      if (!m_running.load())
        return;
      dp = m_dropPoint;
      alt = m_dropAltitude_;
      m_dropPending = false;
    }
    sendDropCommandWithRetry(dp, alt);
    {
      std::lock_guard<std::mutex> lock(m_dropMtx);
      m_dropActive.store(false);
    }
    m_dropDoneCv.notify_all();
  }
}

void MavlinkTelemetry::sendDropCommandWithRetry(Coord dropPointLocal, float altitude)
{
  double latDeg = 0.0, lonDeg = 0.0;
  localToGeo(dropPointLocal.x, dropPointLocal.y, latDeg, lonDeg);

  {
    std::lock_guard<std::mutex> lock(m_ackMtx);
    m_expectedCommand = MAV_CMD_USER_1;
    m_ackReceived = false;
    m_waitingAck = true;
  }

  bool got = false;
  uint8_t result = 0;

  for (int attempt = 1; attempt <= kDropMaxAttempts && m_running.load(); ++attempt) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(m_sysId,
                                  kCompId,
                                  &msg,
                                  0,
                                  0,
                                  MAV_CMD_USER_1,
                                  static_cast<uint8_t>(attempt - 1),
                                  0.0f,
                                  0.0f,
                                  0.0f,
                                  0.0f,
                                  static_cast<float>(latDeg),
                                  static_cast<float>(lonDeg),
                                  altitude);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    sendBuffer(buf, len);

    APP_LOG_MOD("Mavlink", "autopilot: payload drop command transmitted (attempt {}/{})", attempt, kDropMaxAttempts);

    std::unique_lock<std::mutex> lock(m_ackMtx);
    got = m_ackCv.wait_for(lock, std::chrono::milliseconds(kDropAckTimeoutMs), [this] { return m_ackReceived || !m_running.load(); });
    got = got && m_ackReceived;
    if (got) {
      result = m_ackResult;
      break;
    }
  }

  {
    std::lock_guard<std::mutex> lock(m_ackMtx);
    m_waitingAck = false;
  }

  if (got) {
    APP_LOG_MOD("Mavlink", "autopilot: payload command acknowledged successfully (result={})", static_cast<int>(result));
  }
  else {
    APP_LOG_MOD("Mavlink", "autopilot: payload command timeout after {} attempts", kDropMaxAttempts);
  }
}

void MavlinkTelemetry::rxLoop()
{
  mavlink_status_t status;
  std::memset(&status, 0, sizeof(status));
  uint8_t buf[512];

  while (m_running.load()) {
    int n = m_sock.recv(buf, sizeof(buf), 200);
    if (n <= 0) {
      if (!m_running.load())
        break;
      continue;
    }

    for (int i = 0; i < n; ++i) {
      mavlink_message_t msg;
      if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status) != 0) {
        if (msg.msgid != MAVLINK_MSG_ID_COMMAND_ACK)
          continue;

        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(&msg, &ack);

        std::lock_guard<std::mutex> lock(m_ackMtx);
        if (m_waitingAck && ack.command == m_expectedCommand) {
          m_ackReceived = true;
          m_ackResult = ack.result;
          m_ackCv.notify_all();
        }
      }
    }
  }
}

}  // namespace BallisticApp
