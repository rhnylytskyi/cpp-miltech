#include "BallisticApp/mission/ScenarioPathResolver.h"
#include <format>
#include <stdexcept>
#include <algorithm>

namespace BallisticApp {

ScenarioPathResolver::ScenarioPathResolver(std::span<const char* const> args)
{
  resolve(args);
}

void ScenarioPathResolver::resolve(std::span<const char* const> args)
{
  std::string_view scenarioArg = "";

  auto it = std::find_if(args.begin(), args.end(), [](std::string_view arg) { return arg == "--scenario" || arg == "-s"; });

  if (it != args.end()) {
    if (std::next(it) == args.end()) {
      throw std::runtime_error("Error: Missing value for --scenario / -s argument!");
    }
    scenarioArg = *std::next(it);
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

void ScenarioPathResolver::validate() const
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

const std::filesystem::path& ScenarioPathResolver::getConfigPath() const
{
  return m_configPath;
}

const std::filesystem::path& ScenarioPathResolver::getTargetsPath() const
{
  return m_targetsPath;
}

const std::filesystem::path& ScenarioPathResolver::getAmmoPath() const
{
  return m_ammoPath;
}

const std::filesystem::path& ScenarioPathResolver::getSimulationPath() const
{
  return m_simulationPath;
}

}  // namespace BallisticApp
