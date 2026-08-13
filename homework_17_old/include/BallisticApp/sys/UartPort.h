#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace BallisticApp::sys {

/**
 * @brief RAII wrapper around a POSIX serial port file descriptor.
 */
class UartPort {
public:
  UartPort() = default;
  ~UartPort() noexcept;

  /* Unique hardware resource management */
  UartPort(const UartPort &) = delete;
  UartPort &operator=(const UartPort &) = delete;

  /* Port lifecycle management */
  void open(const std::string &device);
  void close() noexcept;
  [[nodiscard]] bool isOpen() const noexcept { return m_fd >= 0; }

  /* Non-blocking data transmission interfaces */
  [[nodiscard]] int read(uint8_t *buf, size_t maxLen);
  void write(const uint8_t *buf, size_t len);

private:
  int m_fd = -1;
};

}  // namespace BallisticApp::sys
