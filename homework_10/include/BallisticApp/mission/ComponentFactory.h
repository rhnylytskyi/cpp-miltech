#pragma once

#include "BallisticApp/solvers/SolverType.h"
#include <filesystem>
#include <memory>

namespace BallisticApp {

class IBallisticSolver;
class IConfigLoader;
class ISimulationExporter;

enum class ConfigLoaderType { FILE };
enum class ExporterType { JSON };

class ComponentFactory {
public:
  ComponentFactory() = delete;

  static std::unique_ptr<IBallisticSolver> createSolver(SolverType type);
  static std::unique_ptr<IConfigLoader> createLoader(ConfigLoaderType type);
  static std::unique_ptr<ISimulationExporter> createExporter(ExporterType type, const std::filesystem::path& param);
};

}  // namespace BallisticApp
