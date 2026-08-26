#include "BallisticApp/link/UartPort.h"
#include <chrono>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace BallisticApp {

UartPort::~UartPort() noexcept
{
  close();
}

void UartPort::open(const std::string& device)
{
  close();

  m_fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (m_fd < 0) {
    throw std::runtime_error("UartPort: cannot open '" + device + "': " + std::strerror(errno));
  }

  termios tio{};

  if (tcgetattr(m_fd, &tio) != 0) {
    throw std::runtime_error("UartPort: tcgetattr failed: " + std::string(std::strerror(errno)));
  }

  cfmakeraw(&tio);
  cfsetispeed(&tio, B115200);
  cfsetospeed(&tio, B115200);
  tio.c_cflag |= (CLOCAL | CREAD);

  if (tcsetattr(m_fd, TCSANOW, &tio) != 0) {
    throw std::runtime_error("UartPort: tcsetattr failed: " + std::string(std::strerror(errno)));
  }
}

void UartPort::close() noexcept
{
  if (m_fd >= 0) {
    ::close(m_fd);
    m_fd = -1;
  }
}

int UartPort::read(uint8_t* buf, size_t maxLen)
{
  if (m_fd < 0) {
    throw std::runtime_error("UartPort::read: port not open");
  }

  ssize_t n = ::read(m_fd, buf, maxLen);

  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return 0;
    }

    throw std::runtime_error("UartPort::read failed: " + std::string(std::strerror(errno)));
  }

  return static_cast<int>(n);
}

void UartPort::write(const uint8_t* buf, size_t len)
{
  if (m_fd < 0) {
    throw std::runtime_error("UartPort::write: port not open");
  }

  size_t off = 0;

  while (off < len) {
    ssize_t n = ::write(m_fd, buf + off, len - off);

    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        continue;
      }

      throw std::runtime_error("UartPort::write failed: " + std::string(std::strerror(errno)));
    }

    off += static_cast<size_t>(n);
  }
}

}  // namespace BallisticApp
