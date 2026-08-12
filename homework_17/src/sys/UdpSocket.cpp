#include "BallisticApp/sys/UdpSocket.h"
#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <netdb.h>
#include <poll.h>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>

namespace BallisticApp::sys {

UdpSocket::~UdpSocket() noexcept
{
  close();
}

void UdpSocket::open(const std::string& host, uint16_t port)
{
  close();

  m_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (m_fd < 0) {
    throw std::runtime_error("UdpSocket: open failed: " + std::string(std::strerror(errno)));
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);

  // Resolve hostname or raw IP
  if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    addrinfo* res = nullptr;

    if (::getaddrinfo(host.c_str(), nullptr, &hints, &res) != 0 || !res) {
      ::close(m_fd);
      m_fd = -1;
      throw std::runtime_error("UdpSocket: resolve failed for " + host);
    }

    addr.sin_addr = reinterpret_cast<sockaddr_in*>(res->ai_addr)->sin_addr;
    ::freeaddrinfo(res);
  }

  // Connect binds the default remote address for send/recv
  if (::connect(m_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    int err = errno;
    ::close(m_fd);
    m_fd = -1;
    throw std::runtime_error("UdpSocket: connect failed: " + std::string(std::strerror(err)));
  }
}

void UdpSocket::close() noexcept
{
  if (m_fd >= 0) {
    ::close(m_fd);
    m_fd = -1;
  }
}

void UdpSocket::send(const uint8_t* buf, size_t len) noexcept
{
  if (m_fd >= 0) {
    static_cast<void>(::send(m_fd, buf, len, 0));
  }
}

int UdpSocket::recv(uint8_t* buf, size_t maxLen) noexcept
{
  if (m_fd < 0) {
    return 0;
  }

  // Non-blocking check using poll with 0ms timeout
  pollfd pfd{m_fd, POLLIN, 0};
  int pr = ::poll(&pfd, 1, 0);
  if (pr <= 0) {
    return 0;
  }

  ssize_t n = ::recv(m_fd, buf, maxLen, 0);
  return n > 0 ? static_cast<int>(n) : 0;
}

}  // namespace BallisticApp::sys
