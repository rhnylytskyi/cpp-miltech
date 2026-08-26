#pragma once

#include "BallisticApp/net/UdpSocket.h"
#include "BallisticApp/utils/MathUtils.h"
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

namespace BallisticApp {

class MavlinkTelemetry {
public:
  MavlinkTelemetry(std::string host = "127.0.0.1", uint16_t port = 14550, uint8_t sysId = 1);
  ~MavlinkTelemetry();

  MavlinkTelemetry(const MavlinkTelemetry&) = delete;
  MavlinkTelemetry& operator=(const MavlinkTelemetry&) = delete;

  void start();
  void stop();

  static void localToGeo(float x, float y, double& latDeg, double& lonDeg);
  void feedTelemetry(Coord pos, Coord speed, float direction, float altitude, uint32_t simTimeMs);
  void notifyDrop(Coord dropPointLocal, float altitude);
  void waitDropSettled();

  [[nodiscard]] bool isAckReceived() const noexcept { return m_ackReceived; }

private:
  void heartbeatLoop();
  void commandLoop();
  void rxLoop();

  void sendDropCommandWithRetry(Coord dropPointLocal, float altitude);
  void sendBuffer(const uint8_t* buf, uint16_t len);

  std::string m_host;
  uint16_t m_port;
  uint8_t m_sysId;
  UdpSocket m_sock;

  std::atomic<bool> m_running{false};
  std::thread m_heartbeatThread;
  std::thread m_commandThread;
  std::thread m_rxThread;

  std::mutex m_dropMtx;
  std::condition_variable m_dropCv;
  std::condition_variable m_dropDoneCv;
  Coord m_dropPoint{0.0f, 0.0f};
  float m_dropAltitude_{0.0f};
  bool m_dropPending{false};
  std::atomic<bool> m_dropActive{false};

  std::mutex m_ackMtx;
  std::condition_variable m_ackCv;
  uint16_t m_expectedCommand{0};
  uint8_t m_ackResult{0};
  bool m_ackReceived{false};
  bool m_waitingAck{false};
};

}  // namespace BallisticApp
