#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace BallisticApp {

class I2cDevice {
public:
  I2cDevice(const std::string& busPath, int address);
  ~I2cDevice();

  I2cDevice(const I2cDevice&) = delete;
  I2cDevice& operator=(const I2cDevice&) = delete;

  [[nodiscard]] std::vector<std::uint8_t> readRegisters(std::uint8_t reg, std::size_t count) const;
  [[nodiscard]] std::uint8_t readRegister(std::uint8_t reg) const;
  void writeRegister(std::uint8_t reg, std::uint8_t value) const;

private:
  int m_fd = -1;
  std::string m_busPath;
  int m_address;
};

}  // namespace BallisticApp
