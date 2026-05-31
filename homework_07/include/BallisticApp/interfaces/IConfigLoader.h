#pragma once

#include "BallisticApp/types/DroneConfig.h"
#include "BallisticApp/types/AmmoParams.h"
#include <filesystem>

namespace BallisticApp {

class IConfigLoader {
public:
  virtual ~IConfigLoader() = default;

  virtual void load(const std::filesystem::path& configPath, const std::filesystem::path& ammoPath) = 0;

  virtual const DroneConfig& getConfig() const = 0;
  virtual const AmmoParams& getAmmoParams() const = 0;
};

}  // namespace BallisticApp
