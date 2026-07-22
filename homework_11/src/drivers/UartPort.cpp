#include "BallisticApp/drivers/UartPort.h"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

namespace BallisticApp::sys {

UartPort::~UartPort()
{
  close();
}

void UartPort::open(const std::string &device)
{
  close();

  // O_NONBLOCK: наступні read() повертають -1/EAGAIN замість блокування
  m_fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (m_fd < 0) {
    throw std::runtime_error("UartPort: cannot open '" + device + "': " + std::strerror(errno));
  }

  termios tio{};

  if (tcgetattr(m_fd, &tio) != 0) {
    throw std::runtime_error("UartPort: tcgetattr failed: " + std::string(std::strerror(errno)));
  }

  cfmakeraw(&tio);  // 8N1, без обробки символів (raw mode)
  cfsetispeed(&tio, B115200);
  cfsetospeed(&tio, B115200);
  tio.c_cflag |= (CLOCAL | CREAD);

  if (tcsetattr(m_fd, TCSANOW, &tio) != 0) {
    throw std::runtime_error("UartPort: tcsetattr failed: " + std::string(std::strerror(errno)));
  }
}

void UartPort::close()
{
  if (m_fd >= 0) {
    ::close(m_fd);
    m_fd = -1;
  }
}

int UartPort::read(uint8_t *buf, size_t maxLen)
{
  if (m_fd < 0) {
    throw std::runtime_error("UartPort::read: port not open");
  }

  ssize_t n = ::read(m_fd, buf, maxLen);

  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return 0;  // даних поки нема
    }

    throw std::runtime_error("UartPort::read failed: " + std::string(std::strerror(errno)));
  }

  return (int)n;
}

void UartPort::write(const uint8_t *buf, size_t len)
{
  if (m_fd < 0) {
    throw std::runtime_error("UartPort::write: port not open");
  }

  size_t off = 0;

  while (off < len) {
    ssize_t n = ::write(m_fd, buf + off, len - off);

    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }

      throw std::runtime_error("UartPort::write failed: " + std::string(std::strerror(errno)));
    }

    off += (size_t)n;
  }
}

}  // namespace BallisticApp::sys
