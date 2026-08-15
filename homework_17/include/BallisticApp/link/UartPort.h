#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace BallisticApp {

class UartPort {
public:
  UartPort() = default;
  ~UartPort() noexcept;

  UartPort(const UartPort&) = delete;
  UartPort& operator=(const UartPort&) = delete;

  void open(const std::string& device);
  void close() noexcept;
  [[nodiscard]] bool isOpen() const noexcept { return m_fd >= 0; }

  [[nodiscard]] int read(uint8_t* buf, size_t maxLen);
  void write(const uint8_t* buf, size_t len);

private:
  int m_fd = -1;
};

}  // namespace BallisticApp
