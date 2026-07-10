#include "BallisticApp/mission/AppArguments.h"
#include <format>
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <string_view>

namespace BallisticApp {

AppArguments::AppArguments(std::span<const char* const> args)
{
  parse(args);
}

void AppArguments::printHelp() const
{
  std::cout << "===================================================================\n"
            << "  BallisticApp Drone Simulation Command Line Interface (CLI)\n"
            << "===================================================================\n"
            << "Usage:\n"
            << "  ./ballistic_app [options]\n\n"
            << "Options:\n"
            << "  -s, --scenario <path>   Specify path to the scenario folder directory\n"
            << "  --enable-lock           Enable persistent weapon-system lock on a single target\n"
            << "                          (Default: disabled, using dynamic multi-target scan)\n"
            << "  --solver <type>         Select solver engine: 'analytical' or 'table'\n"
            << "                          (Default: analytical)\n"
            << "  -h, --help              Display this help menu documentation and exit\n"
            << "===================================================================\n";
}

void AppArguments::parse(std::span<const char* const> args)
{
  // 1. Intercept help command arguments instantly using standard search predicates
  auto helpIt = std::find_if(args.begin(), args.end(), [](std::string_view arg) { return arg == "--help" || arg == "-h"; });

  if (helpIt != args.end()) {
    printHelp();
    std::exit(0);  // Graceful termination upon manual documentation request
  }

  // 2. Parse binary weapon target tracker lock persistence override flag
  auto lockIt = std::find_if(args.begin(), args.end(), [](std::string_view arg) { return arg == "--enable-lock"; });

  m_enableTargetLock = (lockIt != args.end());

  // 2b. Parse selected ballistic solver type (--solver table / analytical)
  auto solverIt = std::find_if(args.begin(), args.end(), [](std::string_view arg) { return arg == "--solver"; });
  if (solverIt != args.end()) {
    if (std::next(solverIt) == args.end()) {
      throw std::runtime_error("Error: Missing associated value for --solver parameter argument!");
    }
    std::string_view solverTypeStr = *std::next(solverIt);
    if (solverTypeStr == "table") {
      m_solverType = SolverType::TABLE;
    }
    else {
      m_solverType = SolverType::ANALYTICAL;
    }
  }

  // 3. Extract the requested target scenario folder input path context
  std::string_view scenarioArg = "";
  auto scenarioIt = std::find_if(args.begin(), args.end(), [](std::string_view arg) { return arg == "--scenario" || arg == "-s"; });

  if (scenarioIt != args.end()) {
    // Structural boundary condition validation for key-value argument strings
    if (std::next(scenarioIt) == args.end()) {
      throw std::runtime_error("Error: Missing associated directory value path for --scenario / -s parameter argument!");
    }
    scenarioArg = *std::next(scenarioIt);
  }

  // 4. Resolve global input/output data package file environments
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
  m_simulationPath = m_dataDir / "simulation.json";
  m_tablePath = m_dataDir / "ballistic_table.txt";

  if (m_solverType == SolverType::TABLE) {
    m_ammoPath = m_dataDir / "ammo_table.json";
  }
  else {
    m_ammoPath = m_dataDir / "ammo.json";
  }

  validate();
}

void AppArguments::validate() const
{
  std::string missingFiles;

  if (!std::filesystem::exists(m_configPath))
    missingFiles += std::format(" - Config: {}\n", m_configPath.string());
  if (!std::filesystem::exists(m_targetsPath))
    missingFiles += std::format(" - Targets: {}\n", m_targetsPath.string());
  if (!std::filesystem::exists(m_ammoPath))
    missingFiles += std::format(" - Ammo: {}\n", m_ammoPath.string());

  if (m_solverType == SolverType::TABLE && !std::filesystem::exists(m_tablePath)) {
    missingFiles += std::format(" - Ballistic Table: {}\n", m_tablePath.string());
  }

  if (!missingFiles.empty()) {
    throw std::runtime_error(std::format("Error: Required input configuration files not found!\nMissing paths:\n{}", missingFiles));
  }
}

const std::filesystem::path& AppArguments::getConfigPath() const
{
  return m_configPath;
}

const std::filesystem::path& AppArguments::getTargetsPath() const
{
  return m_targetsPath;
}

const std::filesystem::path& AppArguments::getAmmoPath() const
{
  return m_ammoPath;
}

const std::filesystem::path& AppArguments::getSimulationPath() const
{
  return m_simulationPath;
}

const std::filesystem::path& AppArguments::getTablePath() const
{
  return m_tablePath;
}

}  // namespace BallisticApp
