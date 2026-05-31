#pragma once

#include <filesystem>
#include <span>

namespace BallisticApp {

class ScenarioPathResolver {
public:
  explicit ScenarioPathResolver(std::span<const char* const> args);

private:
  void resolve(std::span<const char* const> args);
  void validate() const;

public:
  const std::filesystem::path& getConfigPath() const;
  const std::filesystem::path& getTargetsPath() const;
  const std::filesystem::path& getAmmoPath() const;
  const std::filesystem::path& getSimulationPath() const;

private:
  std::filesystem::path m_configDir;
  std::filesystem::path m_dataDir;

  std::filesystem::path m_configPath;
  std::filesystem::path m_targetsPath;
  std::filesystem::path m_ammoPath;
  std::filesystem::path m_simulationPath;
};

}  // namespace BallisticApp
