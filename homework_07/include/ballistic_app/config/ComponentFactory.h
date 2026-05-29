#pragma once

#include "ballistic_app/interfaces/IBallisticSolver.h"
#include "ballistic_app/interfaces/ITargetProvider.h"
#include "ballistic_app/interfaces/IConfigLoader.h"
#include "ballistic_app/interfaces/ISimulationExporter.h"
#include <string>
#include <memory>

namespace BallisticApp {

enum class SolverType { ANALYTICAL };
enum class TargetProviderType { JSON };
enum class ConfigLoaderType { FILE };
enum class ExporterType { JSON };

class ComponentFactory {
public:
  ComponentFactory() = delete;

  static std::unique_ptr<IBallisticSolver> createSolver(SolverType type);
  static std::unique_ptr<ITargetProvider> createProvider(TargetProviderType type, const std::string& param);
  static std::unique_ptr<IConfigLoader> createLoader(ConfigLoaderType type);
  static std::unique_ptr<ISimulationExporter> createExporter(ExporterType type, const std::string& param);
};

}  // namespace BallisticApp
