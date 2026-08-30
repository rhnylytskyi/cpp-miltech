#include "BallisticApp/hardware/Mpu6050.h"
#include <stdexcept>
#include <format>

namespace {

constexpr std::uint8_t kRegWhoAmI = 0x75;
constexpr std::uint8_t kExpectedWhoAmI = 0x68;
constexpr std::uint8_t kRegPwrMgmt1 = 0x6B;
constexpr std::uint8_t kRegAccelXoutH = 0x3B;
constexpr std::size_t kSampleBytes = 14;

// MPU-6050 default scale sensitivity factors
constexpr float kAccelSensitivity = 16384.0F;
constexpr float kGyroSensitivity = 131.0F;

// Construct 16-bit signed integer from Big Endian bytes
std::int16_t combineBigEndian(std::uint8_t high, std::uint8_t low) noexcept
{
  return static_cast<std::int16_t>((static_cast<std::uint16_t>(high) << 8) | low);
}

}  // namespace

namespace BallisticApp {

Mpu6050::Mpu6050(I2cDevice& device)
  : m_device(device)
{
}

void Mpu6050::checkWhoAmI() const
{
  const auto whoAmI = m_device.readRegister(kRegWhoAmI);
  if (whoAmI != kExpectedWhoAmI) {
    throw std::runtime_error(std::format("Invalid WHO_AM_I: expected 0x{:02X}, received 0x{:02X}", kExpectedWhoAmI, whoAmI));
  }
}

void Mpu6050::wake() const
{
  m_device.writeRegister(kRegPwrMgmt1, 0x00);
}

Mpu6050Sample Mpu6050::readSample() const
{
  // Burst read 14 bytes: accel(6) + temp(2) + gyro(6)
  const auto raw = m_device.readRegisters(kRegAccelXoutH, kSampleBytes);

  const std::int16_t rawAccelX = combineBigEndian(raw[0], raw[1]);
  const std::int16_t rawAccelY = combineBigEndian(raw[2], raw[3]);
  const std::int16_t rawAccelZ = combineBigEndian(raw[4], raw[5]);
  const std::int16_t rawTemp = combineBigEndian(raw[6], raw[7]);
  const std::int16_t rawGyroX = combineBigEndian(raw[8], raw[9]);
  const std::int16_t rawGyroY = combineBigEndian(raw[10], raw[11]);
  const std::int16_t rawGyroZ = combineBigEndian(raw[12], raw[13]);

  Mpu6050Sample sample;
  sample.accelX_g = static_cast<float>(rawAccelX) / kAccelSensitivity;
  sample.accelY_g = static_cast<float>(rawAccelY) / kAccelSensitivity;
  sample.accelZ_g = static_cast<float>(rawAccelZ) / kAccelSensitivity;

  sample.gyroX_dps = static_cast<float>(rawGyroX) / kGyroSensitivity;
  sample.gyroY_dps = static_cast<float>(rawGyroY) / kGyroSensitivity;
  sample.gyroZ_dps = static_cast<float>(rawGyroZ) / kGyroSensitivity;

  // Official MPU-6050 datasheet formula
  sample.temperatureC = static_cast<float>(rawTemp) / 340.0F + 36.53F;

  return sample;
}

}  // namespace BallisticApp
