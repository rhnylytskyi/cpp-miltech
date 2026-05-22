#pragma once

#include <ballistic_app/interfaces/IConfigLoader.hpp>
#include <ballistic_app/dto/DroneConfig.hpp>
#include <ballistic_app/dto/AmmoParams.hpp>
#include <string>

namespace BallisticApp {

class FileConfigLoader : public IConfigLoader {
private:
  DroneConfig config;
  AmmoParams ammo;

  void loadAmmoParams(const char* ammoPath, const std::string& targetAmmoName);

public:
  FileConfigLoader();
  ~FileConfigLoader() override = default;

  void load(const char* configPath, const char* ammoSource) override;
  DroneConfig getConfig() override;
  AmmoParams getAmmoParams() override;
};

}  // namespace BallisticApp
