#include "BallisticApp/ConfigLocator.h"
#include <format>
#include <stdexcept>
#include <string_view>

namespace BallisticApp {

ConfigLocator::ConfigLocator(int argc, char* argv[])
{
  initialize(argc, argv);
}

void ConfigLocator::initialize(int argc, char* argv[])
{
  std::string_view scenarioArg = "";

  // Шукаємо прапорці --scenario або -s
  for (int i = 1; i < argc; ++i) {
    std::string_view arg(argv[i]);
    if ((arg == "--scenario" || arg == "-s") && (i + 1 < argc)) {
      scenarioArg = argv[i + 1];
      break;
    }
  }

  if (!scenarioArg.empty()) {
    std::filesystem::path inputPath(scenarioArg);

    if (inputPath.is_absolute()) {
      m_configDir = inputPath;
      m_dataDir = inputPath.parent_path();
    }
    else {
      m_dataDir = "data";
      if (!inputPath.empty() && *inputPath.begin() == "data") {
        m_configDir = inputPath;
      }
      else {
        m_configDir = m_dataDir / inputPath;
      }
    }
  }

  m_configPath = m_configDir / "config.json";
  m_targetsPath = m_configDir / "targets.json";
  m_ammoPath = m_dataDir / "ammo.json";
  m_simulationPath = m_dataDir / "simulation.json";

  validate();
}

void ConfigLocator::validate() const
{
  std::string missingFiles;

  if (!std::filesystem::exists(m_configPath))
    missingFiles += std::format(" - Config: {}\n", m_configPath.string());
  if (!std::filesystem::exists(m_targetsPath))
    missingFiles += std::format(" - Targets: {}\n", m_targetsPath.string());
  if (!std::filesystem::exists(m_ammoPath))
    missingFiles += std::format(" - Ammo: {}\n", m_ammoPath.string());

  if (!missingFiles.empty()) {
    throw std::runtime_error(std::format("Error: Required input configuration files not found!\nMissing paths:\n{}", missingFiles));
  }
}

const std::filesystem::path& ConfigLocator::getConfigPath() const
{
  return m_configPath;
}

const std::filesystem::path& ConfigLocator::getTargetsPath() const
{
  return m_targetsPath;
}

const std::filesystem::path& ConfigLocator::getAmmoPath() const
{
  return m_ammoPath;
}

const std::filesystem::path& ConfigLocator::getSimulationPath() const
{
  return m_simulationPath;
}

}  // namespace BallisticApp
