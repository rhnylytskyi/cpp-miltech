#pragma once

#include <string>

namespace BallisticApp {
class PathResolver {
public:
  PathResolver(int argc, char* argv[])
  {
    targetsStr = TARGETS_PATH;
    configStr = CONFIG_PATH;

    if (argc > 1) {
      std::string fileName = argv[1];
      targetsStr = "data/test/" + fileName;

      std::string configFileName = fileName;
      if (configFileName.rfind("targets", 0) == 0) {
        configFileName.replace(0, 7, "config");
      }
      configStr = "data/test/" + configFileName;
    }
  }

  const char* getTargetsPath() const { return targetsStr.c_str(); }

  const char* getConfigPath() const { return configStr.c_str(); }

private:
  std::string targetsStr;
  std::string configStr;
};
}  // namespace BallisticApp