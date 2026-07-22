#include "BallisticApp/link/UartPort.h"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

UartPort::~UartPort()
{
  close();
}

void UartPort::open(const std::string &device)
{
  close();

  // O_NONBLOCK: наступні read() повертають -1/EAGAIN замість блокування,
  // коли даних ще нема — дозволяє в тому ж циклі перевіряти GPIO/таймаути.
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (fd_ < 0) {
    throw std::runtime_error("UartPort: cannot open '" + device + "': " + std::strerror(errno));
  }

  termios tio{};

  if (tcgetattr(fd_, &tio) != 0) {
    throw std::runtime_error("UartPort: tcgetattr failed: " + std::string(std::strerror(errno)));
  }

  cfmakeraw(&tio);  // 8N1, без обробки символів (raw mode)
  cfsetispeed(&tio, B115200);
  cfsetospeed(&tio, B115200);  // швидкість з обох боків однакова
  tio.c_cflag |= (CLOCAL | CREAD);

  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    throw std::runtime_error("UartPort: tcsetattr failed: " + std::string(std::strerror(errno)));
  }
}

void UartPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

int UartPort::read(uint8_t *buf, size_t maxLen)
{
  if (fd_ < 0) {
    throw std::runtime_error("UartPort::read: port not open");
  }

  ssize_t n = ::read(fd_, buf, maxLen);

  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return 0;  // даних поки нема — це не помилка з O_NONBLOCK
    }

    throw std::runtime_error("UartPort::read failed: " + std::string(std::strerror(errno)));
  }

  return (int)n;
}

void UartPort::write(const uint8_t *buf, size_t len)
{
  if (fd_ < 0) {
    throw std::runtime_error("UartPort::write: port not open");
  }

  size_t off = 0;

  while (off < len) {
    ssize_t n = ::write(fd_, buf + off, len - off);

    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;  // вихідний буфер тимчасово повний — повторити
      }

      throw std::runtime_error("UartPort::write failed: " + std::string(std::strerror(errno)));
    }

    off += (size_t)n;
  }
}
