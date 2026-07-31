#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace BallisticApp::sys {

/**
 * @brief Lightweight RAII wrapper around a POSIX serial port file descriptor.
 * Enforces non-blocking execution profiles and raw 115200 8N1 transmission.
 */
class UartPort {
public:
  UartPort() = default;
  ~UartPort() noexcept;

  /* Strict resource gating: serial file descriptors must not be duplicated */
  UartPort(const UartPort &) = delete;
  UartPort &operator=(const UartPort &) = delete;

  /**
   * @brief Opens the device path and configures terms for 115200 8N1 raw execution.
   * @throws std::runtime_error if terminal attribute reading or assignment fails.
   */
  void open(const std::string &device);

  /**
   * @brief Cleanly closes the active file descriptor context.
   */
  void close() noexcept;

  [[nodiscard]] bool isOpen() const noexcept { return m_fd >= 0; }

  /**
   * @brief Executes a non-blocking read operation into the provided buffer destination.
   * @return Number of raw bytes read, or 0 if no data is currently available in the hardware buffer.
   */
  [[nodiscard]] int read(uint8_t *buf, size_t maxLen);

  /**
   * @brief Writes exactly len bytes to the serial line, loop-retrying on partial transmissions.
   */
  void write(const uint8_t *buf, size_t len);

private:
  int m_fd = -1;
};

}  // namespace BallisticApp::sys
