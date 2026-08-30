#include "BallisticApp/hardware/I2cDevice.h"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <unistd.h>
#include <format>

namespace {

std::string getErrnoText()
{
  return std::strerror(errno);
}

}  // namespace

namespace BallisticApp {

I2cDevice::I2cDevice(const std::string& busPath, int address)
  : m_busPath(busPath)
  , m_address(address)
{
  m_fd = ::open(m_busPath.c_str(), O_RDWR);
  if (m_fd < 0) {
    throw std::runtime_error(std::format("Failed to open I2C bus {}: {}", m_busPath, getErrnoText()));
  }

  if (::ioctl(m_fd, I2C_SLAVE, m_address) < 0) {
    const auto reason = getErrnoText();
    ::close(m_fd);
    m_fd = -1;
    throw std::runtime_error(std::format("Failed to select address 0x{:02X} on {}: {}", m_address, m_busPath, reason));
  }
}

I2cDevice::~I2cDevice()
{
  if (m_fd >= 0) {
    ::close(m_fd);
  }
}

std::uint8_t I2cDevice::readRegister(std::uint8_t reg) const
{
  return readRegisters(reg, 1).front();
}

std::vector<std::uint8_t> I2cDevice::readRegisters(std::uint8_t reg, std::size_t count) const
{
  // Phase 1: Write target register address
  if (::write(m_fd, &reg, 1) != 1) {
    throw std::runtime_error(std::format("No ACK from device 0x{:02X} on write reg 0x{:02X}: {}", m_address, reg, getErrnoText()));
  }

  // Phase 2: Read raw bytes sequential data block
  std::vector<std::uint8_t> buffer(count);
  const auto readBytes = ::read(m_fd, buffer.data(), count);

  if (readBytes < 0) {
    throw std::runtime_error(std::format("Read failure from reg 0x{:02X} on device 0x{:02X}: {}", reg, m_address, getErrnoText()));
  }

  if (static_cast<std::size_t>(readBytes) != count) {
    throw std::runtime_error(std::format("Read truncation on reg 0x{:02X}: expected {} bytes, got {}", reg, count, readBytes));
  }

  return buffer;
}

void I2cDevice::writeRegister(std::uint8_t reg, std::uint8_t value) const
{
  const std::uint8_t payload[2] = {reg, value};
  if (::write(m_fd, payload, sizeof(payload)) != static_cast<ssize_t>(sizeof(payload))) {
    throw std::runtime_error(
      std::format("No ACK from device 0x{:02X} on write reg 0x{:02X}=0x{:02X}: {}", m_address, reg, value, getErrnoText()));
  }
}

}  // namespace BallisticApp
