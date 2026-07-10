#pragma once

#include <string>
#include <filesystem>

class PathResolver {
public:
  // Парсинг аргументів командного рядка та налаштування папок
  static void parseArguments(int argc, char* argv[]);

  // Отримання шляхів до файлів
  static std::string getTargetsPath();
  static std::string getConfigPath();
  static std::string getAmmoPath();
  static std::string getSimulationPath();

private:
  static std::filesystem::path m_configDir;
  static std::filesystem::path m_dataDir;
};
