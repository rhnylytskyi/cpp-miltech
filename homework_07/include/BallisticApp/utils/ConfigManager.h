#pragma once

#include <string>
#include <filesystem>

namespace BallisticApp {

class ConfigManager {
public:
  void initialize(int argc, char* argv[]);

  std::string getTargetsPath() const;
  std::string getConfigPath() const;
  std::string getAmmoPath() const;
  std::string getSimulationPath() const;

private:
  void validate() const;

  std::filesystem::path m_dataDir{"data"};
  std::filesystem::path m_configDir{"data"};
};

}  // namespace BallisticApp
