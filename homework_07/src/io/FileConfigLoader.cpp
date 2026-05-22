#include "ballistic_app/io/FileConfigLoader.hpp"
#include <fstream>
#include <format>
#include <stdexcept>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace BallisticApp {

FileConfigLoader::FileConfigLoader()
  : config{}
  , ammo{}
{
}

void FileConfigLoader::load(const char* configPath, const char* ammoSource)
{
  std::ifstream f(configPath);
  if (!f.is_open()) {
    throw std::runtime_error(std::format("Could not open config file: \"{}\"", configPath));
  }

  json j;
  f >> j;
  f.close();

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

  // --- Завантаження параметрів снаряду ---
  loadAmmoParams(ammoSource, config.ammoName);
}

void FileConfigLoader::loadAmmoParams(const char* ammoPath, const std::string& targetAmmoName)
{
  std::ifstream fAmmo(ammoPath);
  if (!fAmmo.is_open()) {
    throw std::runtime_error(std::format("Could not open ammo file: \"{}\"", ammoPath));
  }

  json jAmmoArray;
  fAmmo >> jAmmoArray;
  fAmmo.close();

  for (const auto& item : jAmmoArray) {
    if (item["name"].get<std::string>() == targetAmmoName) {
      ammo.name = targetAmmoName;
      ammo.mass = item["mass"].get<float>();
      ammo.drag = item["drag"].get<float>();
      ammo.lift = item["lift"].get<float>();
      return;
    }
  }

  throw std::runtime_error(std::format("Cannot find ammo parameters for \"{}\" in file \"{}\"!", targetAmmoName, ammoPath));
}

DroneConfig FileConfigLoader::getConfig()
{
  return config;
}

AmmoParams FileConfigLoader::getAmmoParams()
{
  return ammo;
}

}  // namespace BallisticApp
