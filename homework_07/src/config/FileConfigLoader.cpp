#include "BallisticApp/loaders/FileConfigLoader.h"
#include <fstream>
#include <format>
#include <stdexcept>
#include <filesystem>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace BallisticApp {

FileConfigLoader::FileConfigLoader()
  : config{}
{
}

void FileConfigLoader::load(const std::filesystem::path& configPath, const std::filesystem::path& ammoSource)
{
  std::ifstream f(configPath);
  if (!f.is_open()) {
    throw std::runtime_error(std::format("Could not open config file: \"{}\"", configPath.string()));
  }

  json j;
  f >> j;
  f.close();

  validateDroneConfig(j);

  // --- Заповнення конфігурації дрону ---
  config.startPos.x = j["drone"]["position"]["x"];
  config.startPos.y = j["drone"]["position"]["y"];
  config.initialDir = j["drone"]["initialDirection"];
  config.altitude = j["drone"]["altitude"];
  config.attackSpeed = j["drone"]["attackSpeed"];
  config.angularSpeed = j["drone"]["angularSpeed"];
  config.turnThreshold = j["drone"]["turnThreshold"];
  config.accelPath = j["drone"]["accelerationPath"];
  config.hitRadius = j["simulation"]["hitRadius"];
  config.simTimeStep = j["simulation"]["timeStep"];
  config.arrayTimeStep = j["targetArrayTimeStep"];
  config.ammoName = j["ammo"].get<std::string>();

  // --- Завантаження параметрів снарядів ---
  loadAmmoParams(ammoSource);
}

void FileConfigLoader::loadAmmoParams(const std::filesystem::path& ammoPath)
{
  std::ifstream fAmmo(ammoPath);
  if (!fAmmo.is_open()) {
    throw std::runtime_error(std::format("Could not open ammo file: \"{}\"", ammoPath.string()));
  }

  json jAmmoArray;
  fAmmo >> jAmmoArray;
  fAmmo.close();

  if (!jAmmoArray.is_array()) {
    throw std::runtime_error(std::format("Ammo file \"{}\" must contain a JSON array!", ammoPath.string()));
  }

  ammoMap.clear();

  for (const auto& item : jAmmoArray) {
    validateAmmoItem(item, ammoPath);

    std::string name = item["name"].get<std::string>();

    AmmoParams params;
    params.name = name;
    params.mass = item["mass"].get<float>();
    params.drag = item["drag"].get<float>();
    params.lift = item["lift"].get<float>();

    ammoMap[name] = params;
  }
}

void FileConfigLoader::validateDroneConfig(const nlohmann::json& j) const
{
  auto assertHasKey = [&j](const std::string& jsonPointerPath) {
    if (!j.contains(json::json_pointer(jsonPointerPath))) {
      throw std::runtime_error(std::format("Config validation failed! Missing required field: '{}'", jsonPointerPath));
    }
  };

  assertHasKey("/drone/position/x");
  assertHasKey("/drone/position/y");
  assertHasKey("/drone/initialDirection");
  assertHasKey("/drone/altitude");
  assertHasKey("/drone/attackSpeed");
  assertHasKey("/drone/angularSpeed");
  assertHasKey("/drone/turnThreshold");
  assertHasKey("/drone/accelerationPath");
  assertHasKey("/simulation/hitRadius");
  assertHasKey("/simulation/timeStep");
  assertHasKey("/targetArrayTimeStep");
  assertHasKey("/ammo");
}

void FileConfigLoader::validateAmmoItem(const nlohmann::json& item, const std::filesystem::path& ammoPath) const
{
  auto assertHasAmmoKey = [&item, &ammoPath](const std::string& key) {
    if (!item.contains(key)) {
      throw std::runtime_error(
        std::format("Ammo validation failed in file \"{}\"! One of the ammo items is missing the '{}' field.", ammoPath.string(), key));
    }
  };

  assertHasAmmoKey("name");
  assertHasAmmoKey("mass");
  assertHasAmmoKey("drag");
  assertHasAmmoKey("lift");
}

const DroneConfig& FileConfigLoader::getConfig() const
{
  return config;
}

const AmmoParams& FileConfigLoader::getAmmoParams() const
{
  auto it = ammoMap.find(config.ammoName);
  if (it == ammoMap.end()) {
    throw std::runtime_error(std::format("Cannot find ammo parameters for \"{}\" among loaded ammunition types!", config.ammoName));
  }
  return it->second;
}

}  // namespace BallisticApp
