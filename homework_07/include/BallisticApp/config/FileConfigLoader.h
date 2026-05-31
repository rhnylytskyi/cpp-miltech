#pragma once

#include "BallisticApp/interfaces/IConfigLoader.h"
#include "BallisticApp/types/DroneConfig.h"
#include "BallisticApp/types/AmmoParams.h"
#include <string>
#include <filesystem>
#include <map>
#include <nlohmann/json_fwd.hpp>

namespace BallisticApp {

class FileConfigLoader : public IConfigLoader {
public:
  FileConfigLoader();
  void load(const std::filesystem::path& configPath, const std::filesystem::path& ammoSource) override;

  const DroneConfig& getConfig() const override;
  const AmmoParams& getAmmoParams() const override;

private:
  void validateDroneConfig(const nlohmann::json& j) const;
  void validateAmmoItem(const nlohmann::json& item, const std::filesystem::path& ammoPath) const;
  void loadAmmoParams(const std::filesystem::path& ammoPath);

  DroneConfig config;
  std::map<std::string, AmmoParams> ammoMap;
};

}  // namespace BallisticApp
