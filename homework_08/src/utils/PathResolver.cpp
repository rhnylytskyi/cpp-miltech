#include "ballistic_app/utils/PathResolver.h"
#include <string_view>

// За замовчуванням файли шукаються в корені папки "data"
std::filesystem::path PathResolver::m_dataDir = "data";
std::filesystem::path PathResolver::m_configDir = "data";

void PathResolver::parseArguments(int argc, char* argv[])
{
  std::string testArg = "";

  // Шукаємо повний прапорець --test або скорочений -t
  for (int i = 1; i < argc; ++i) {
    std::string_view arg(argv[i]);
    if ((arg == "--test" || arg == "-t") && (i + 1 < argc)) {
      testArg = argv[i + 1];
      break;
    }
  }

  // Якщо тест вказано, коригуємо шляхи
  if (!testArg.empty()) {
    std::filesystem::path inputPath(testArg);

    if (inputPath.is_absolute()) {
      m_configDir = inputPath;
      m_dataDir = inputPath.parent_path();
    }
    else {
      m_dataDir = "data";
      // Перевіряємо, чи користувач не ввів шлях одразу з "data/"
      if (inputPath.string().find("data") == 0) {
        m_configDir = inputPath;
      }
      else {
        m_configDir = m_dataDir / inputPath;  // Отримуємо "data/t01_base"
      }
    }
  }
}

std::string PathResolver::getTargetsPath()
{
  return (m_configDir / "targets.json").string();
}

std::string PathResolver::getConfigPath()
{
  return (m_configDir / "config.json").string();
}

std::string PathResolver::getAmmoPath()
{
  return (m_dataDir / "ammo.json").string();
}

std::string PathResolver::getSimulationPath()
{
  return (m_dataDir / "simulation.json").string();
}
