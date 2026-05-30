#pragma once
#include "BallisticApp/interfaces/IConfigLoader.h"
#include "BallisticApp/DroneConfig.h"
#include "BallisticApp/AmmoParams.h"
#include <string>
#include <map>
#include <nlohmann/json_fwd.hpp>

namespace BallisticApp {

class FileConfigLoader : public IConfigLoader {
public:
  FileConfigLoader();
  void load(const std::string& configPath, const std::string& ammoSource) override;

  DroneConfig getConfig() const override;
  AmmoParams getAmmoParams() const override;

private:
  void validateDroneConfig(const nlohmann::json& j) const;
  void validateAmmoItem(const nlohmann::json& item, const std::string& ammoPath) const;
  void loadAmmoParams(const std::string& ammoPath);

  DroneConfig config;
  std::map<std::string, AmmoParams> ammoMap;
};

}  // namespace BallisticApp
