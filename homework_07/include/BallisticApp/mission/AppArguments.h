#pragma once

#include "BallisticApp/solvers/SolverType.h"
#include <filesystem>
#include <span>

namespace BallisticApp {

/**
 * @brief Parses and stores all command-line arguments passed to the application.
 * Handles scenario data directories, configuration paths, and tactical logic flags.
 */
class AppArguments {
public:
  /**
   * @brief Dispatches command line arguments using core standard library tools.
   * @param args Array span containing input terminal strings.
   */
  explicit AppArguments(std::span<const char* const> args);

  const std::filesystem::path& getConfigPath() const;

  const std::filesystem::path& getTargetsPath() const;

  const std::filesystem::path& getAmmoPath() const;

  const std::filesystem::path& getSimulationPath() const;

  const std::filesystem::path& getTablePath() const;

  /**
   * @brief Verifies whether runtime target tracker lock persistence is enabled.
   * @return True if --enable-lock requested, false otherwise (default).
   */
  bool isTargetLockEnabled() const noexcept { return m_enableTargetLock; }

  /**
   * @brief Returns the selected ballistic solver type.
   * @return SolverType::ANALYTICAL or SolverType::TABLE.
   */
  SolverType getSolverType() const noexcept { return m_solverType; }

private:
  std::filesystem::path m_dataDir;
  std::filesystem::path m_configDir;

  std::filesystem::path m_configPath;
  std::filesystem::path m_targetsPath;
  std::filesystem::path m_ammoPath;
  std::filesystem::path m_simulationPath;
  std::filesystem::path m_tablePath;

  bool m_enableTargetLock{false};
  SolverType m_solverType{SolverType::ANALYTICAL};

  void parse(std::span<const char* const> args);
  void validate() const;
  void printHelp() const;
};

}  // namespace BallisticApp
