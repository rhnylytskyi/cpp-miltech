#pragma once

#include <netinet/in.h>
#include <cstddef>
#include <cstdint>
#include <string>

namespace BallisticApp {

class UdpSocket {
public:
  UdpSocket() = default;
  ~UdpSocket();

  UdpSocket(const UdpSocket&) = delete;
  UdpSocket& operator=(const UdpSocket&) = delete;

  void open(const std::string& host, uint16_t port);
  void close();
  [[nodiscard]] bool isOpen() const noexcept { return m_fd >= 0; }

  void send(const uint8_t* buf, size_t len);
  int recv(uint8_t* buf, size_t maxLen, int timeoutMs);

private:
  int m_fd = -1;
  sockaddr_in m_remoteAddr{};
};

}  // namespace BallisticApp
