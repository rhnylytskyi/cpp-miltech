#pragma once

#include <string>
#include <cstdint>
#include <stddef.h>

namespace BallisticApp::sys {

class UdpSocket {
public:
  UdpSocket() = default;
  ~UdpSocket() noexcept;

  UdpSocket(const UdpSocket&) = delete;
  UdpSocket& operator=(const UdpSocket&) = delete;

  void open(const std::string& host, uint16_t port);
  void close() noexcept;

  void send(const uint8_t* buf, size_t len) noexcept;
  [[nodiscard]] int recv(uint8_t* buf, size_t maxLen) noexcept;

private:
  int m_fd{-1};
};

}  // namespace BallisticApp::sys
