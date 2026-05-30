#pragma once

#include "BallisticApp/DroneConfig.h"
#include "BallisticApp/AmmoParams.h"
#include <string>

namespace BallisticApp {

class IConfigLoader {
public:
  virtual ~IConfigLoader() = default;

  virtual void load(const std::string& configPath, const std::string& ammoPath) = 0;

  virtual DroneConfig getConfig() const = 0;
  virtual AmmoParams getAmmoParams() const = 0;
};

}  // namespace BallisticApp
