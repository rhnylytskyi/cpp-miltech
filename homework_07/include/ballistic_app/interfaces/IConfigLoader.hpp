#pragma once
#include "ballistic_app/dto/DroneConfig.hpp"
#include "ballistic_app/dto/AmmoParams.hpp"

namespace BallisticApp {
class IConfigLoader {
public:
  virtual void load(const char* configPath, const char* ammoPath) = 0;
  virtual DroneConfig getConfig() = 0;
  virtual AmmoParams getAmmoParams() = 0;
  virtual ~IConfigLoader() {}
};
}  // namespace BallisticApp
