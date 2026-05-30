#include "BallisticApp/utils/ConfigManager.h"
#include <string_view>
#include <stdexcept>

namespace BallisticApp {

void ConfigManager::initialize(int argc, char* argv[])
{
  std::string testArg = "";

  for (int i = 1; i < argc; ++i) {
    std::string_view arg(argv[i]);
    if ((arg == "--test" || arg == "-t") && (i + 1 < argc)) {
      testArg = argv[i + 1];
      break;
    }
  }

  if (!testArg.empty()) {
    std::filesystem::path inputPath(testArg);

    if (inputPath.is_absolute()) {
      m_configDir = inputPath;
      m_dataDir = inputPath.parent_path();
    }
    else {
      m_dataDir = "data";
      if (inputPath.string().find("data") == 0) {
        m_configDir = inputPath;
      }
      else {
        m_configDir = m_dataDir / inputPath;
      }
    }
  }

  validate();
}

void ConfigManager::validate() const
{
  if (!std::filesystem::exists(getConfigPath()) || !std::filesystem::exists(getAmmoPath()) || !std::filesystem::exists(getTargetsPath())) {
    std::string errorMsg =
      "Error: Required configuration files not found!\nChecked paths:\n"
      " - Config: " +
      getConfigPath() + "\n" + " - Targets: " + getTargetsPath() + "\n" + " - Ammo: " + getAmmoPath() + "\n" +
      " - Simulation: " + getSimulationPath();

    throw std::runtime_error(errorMsg);
  }
}

std::string ConfigManager::getTargetsPath() const
{
  return (m_configDir / "targets.json").string();
}
std::string ConfigManager::getConfigPath() const
{
  return (m_configDir / "config.json").string();
}
std::string ConfigManager::getAmmoPath() const
{
  return (m_dataDir / "ammo.json").string();
}
std::string ConfigManager::getSimulationPath() const
{
  return (m_dataDir / "simulation.json").string();
}

}  // namespace BallisticApp
