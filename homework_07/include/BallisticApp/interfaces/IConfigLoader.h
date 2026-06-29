#pragma once

#include <filesystem>

namespace BallisticApp {

struct AmmoParams;
struct DroneConfig;

class IConfigLoader {
public:
  virtual ~IConfigLoader() = default;

  virtual void load(const std::filesystem::path& configPath, const std::filesystem::path& ammoPath) = 0;

  virtual const DroneConfig& getConfig() const = 0;
  virtual const AmmoParams& getAmmoParams() const = 0;
};

}  // namespace BallisticApp
