#pragma once

#include <filesystem>

namespace BallisticApp {

class ConfigLocator {
public:
  ConfigLocator(int argc, char* argv[]);

  void validate() const;

  const std::filesystem::path& getConfigPath() const;
  const std::filesystem::path& getTargetsPath() const;
  const std::filesystem::path& getAmmoPath() const;
  const std::filesystem::path& getSimulationPath() const;

private:
  void initialize(int argc, char* argv[]);

  std::filesystem::path m_configPath;
  std::filesystem::path m_targetsPath;
  std::filesystem::path m_ammoPath;
  std::filesystem::path m_simulationPath;
  
  std::filesystem::path m_configDir;
  std::filesystem::path m_dataDir;
};

}  // namespace BallisticApp
