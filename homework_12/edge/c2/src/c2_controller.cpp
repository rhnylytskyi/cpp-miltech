#include "c2_controller.hpp"
#include "fc_link.hpp"     // MAVSDK обгортка, API описано у fc_link.hpp
#include "udp_socket.hpp"  // UDP прийом, API описано у udp_socket.hpp

#include <nlohmann/json.hpp>  // Розбiр JSON з точками маршруту вiд auto_stub

#include <cerrno>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

static constexpr uint16_t STUB_PORT = 14560;
static constexpr const char* LOG_PATH = "/var/log/c2/c2.log";
static constexpr const char* HEALTHY_PATH = "/tmp/c2_healthy";

struct C2Controller::Impl {
  C2State state = C2State::DISARMED;

  FcLink fc;
  UdpSocket stubSocket;
  std::ofstream logFile;

  bool holdSent = false;
  bool healthyWritten = false;

  explicit Impl(uint16_t fcPort)
    : fc(fcPort)
    , stubSocket(STUB_PORT)
    , logFile(LOG_PATH, std::ios::app)
  {
  }

  static const char* stateName(C2State s)
  {
    switch (s) {
      case C2State::DISARMED:
        return "DISARMED";
      case C2State::ARMED_HOLD:
        return "ARMED_HOLD";
      case C2State::ARMED_GUIDED:
        return "ARMED_GUIDED";
      case C2State::ARMED_MANUAL:
        return "ARMED_MANUAL";
    }
    return "UNKNOWN";
  }

  void logLine(const std::string& line)
  {
    std::cout << line << std::endl;
    if (logFile.is_open()) {
      logFile << line << std::endl;
    }
  }

  void transition(C2State next)
  {
    if (next == state) {
      return;
    }

    logLine(std::string("[C2] state: ") + stateName(state) + " -> " + stateName(next));
    state = next;
    holdSent = false;
  }

  void updateHealthcheck()
  {
    if (!healthyWritten && fc.is_connected()) {
      std::ofstream(HEALTHY_PATH).close();
      healthyWritten = true;
    }
  }

  C2State computeNextState() const
  {
    if (!fc.is_armed()) {
      return C2State::DISARMED;
    }

    switch (fc.flight_mode()) {
      case FcLink::FlightMode::Guided:
        return C2State::ARMED_GUIDED;
      case FcLink::FlightMode::Manual:
        return C2State::ARMED_MANUAL;
      case FcLink::FlightMode::Hold:
        return C2State::ARMED_HOLD;
      case FcLink::FlightMode::Unknown:
        return C2State::ARMED_HOLD;
    }

    return C2State::DISARMED;
  }

  void applyHoldAction()
  {
    if (state == C2State::ARMED_HOLD && !holdSent) {
      fc.hold();
      holdSent = true;
    }
  }

  bool receiveWaypoint(double& northM, double& eastM)
  {
    char buf[512];
    ssize_t n = stubSocket.recv(buf, sizeof(buf) - 1);

    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return false;
      }
      logLine(std::string("[C2] error: recvfrom() failed, errno=") + std::to_string(errno));
      return false;
    }

    if (n == 0) {
      return false;
    }

    buf[n] = '\0';

    try {
      auto j = nlohmann::json::parse(buf);
      northM = j.at("north_m").get<double>();
      eastM = j.at("east_m").get<double>();
    }
    catch (const nlohmann::json::exception& e) {
      logLine(std::string("[C2] error: bad waypoint json: ") + e.what());
      return false;
    }

    return true;
  }

  void handleWaypoint(double northM, double eastM)
  {
    if (state == C2State::ARMED_GUIDED) {
      fc.go_to_ned(static_cast<float>(northM), static_cast<float>(eastM));

      std::ostringstream oss;
      oss << "[C2] fwd: north=" << northM << " east=" << eastM;
      logLine(oss.str());
    }
    else {
      logLine(std::string("[C2] blocked: waypoint in ") + stateName(state));
    }
  }

  void processWaypoint()
  {
    double northM = 0.0, eastM = 0.0;
    if (receiveWaypoint(northM, eastM)) {
      handleWaypoint(northM, eastM);
    }
  }
};

C2Controller::C2Controller(uint16_t fc_port)
  : impl_(std::make_unique<Impl>(fc_port))
{
}

C2Controller::~C2Controller() = default;

void C2Controller::tick()
{
  impl_->updateHealthcheck();
  impl_->transition(impl_->computeNextState());
  impl_->applyHoldAction();
  impl_->processWaypoint();
}

C2State C2Controller::current_state() const
{
  return impl_->state;
}
