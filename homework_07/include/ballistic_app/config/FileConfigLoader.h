#pragma once

#include "ballistic_app/interfaces/IConfigLoader.h"
#include "ballistic_app/Types.h"
#include <string>

namespace BallisticApp {

class FileConfigLoader : public IConfigLoader {
private:
  DroneConfig config;
  AmmoParams ammo;

  void loadAmmoParams(const std::string& ammoPath, const std::string& targetAmmoName);

public:
  FileConfigLoader();
  ~FileConfigLoader() override = default;

  void load(const std::string& configPath, const std::string& ammoSource) override;
  DroneConfig getConfig() const override;
  AmmoParams getAmmoParams() const override;
};

}  // namespace BallisticApp
