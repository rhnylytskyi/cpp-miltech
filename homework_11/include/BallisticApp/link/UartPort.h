#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

// Тонка RAII-обгортка над файловим дескриптором послідовного порту.
// Той самий код працює і для /tmp/ttyA (socat pty, симулятор),
// і для /dev/ttyAMA1 (реальна плата) — відрізняється лише шлях у --uart.
class UartPort {
public:
  UartPort() = default;
  ~UartPort();

  UartPort(const UartPort &) = delete;
  UartPort &operator=(const UartPort &) = delete;

  // Відкрити порт, налаштувати 115200 8N1 raw, O_NONBLOCK.
  // Кидає std::runtime_error при помилці.
  void open(const std::string &device);

  void close();
  bool isOpen() const { return fd_ >= 0; }

  // Неблокуюче читання. Повертає кількість прочитаних байтів (0 якщо даних нема).
  // Кидає std::runtime_error лише на "справжній" помилці (не EAGAIN/EWOULDBLOCK).
  int read(uint8_t *buf, size_t maxLen);

  // Записати рівно len байтів (перезапускає write() на часткових записах).
  void write(const uint8_t *buf, size_t len);

private:
  int fd_ = -1;
};
