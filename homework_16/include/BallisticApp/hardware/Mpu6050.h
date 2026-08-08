#pragma once

#include "BallisticApp/hardware/I2cDevice.h"

namespace BallisticApp {

struct Mpu6050Sample {
  float accelX_g = 0.0F;
  float accelY_g = 0.0F;
  float accelZ_g = 0.0F;
  float gyroX_dps = 0.0F;
  float gyroY_dps = 0.0F;
  float gyroZ_dps = 0.0F;
  float temperatureC = 0.0F;
};

class Mpu6050 {
public:
  static constexpr int kDefaultAddress = 0x68;

  explicit Mpu6050(I2cDevice& device);

  void checkWhoAmI() const;
  void wake() const;

  [[nodiscard]] Mpu6050Sample readSample() const;

private:
  I2cDevice& m_device;
};

}  // namespace BallisticApp
