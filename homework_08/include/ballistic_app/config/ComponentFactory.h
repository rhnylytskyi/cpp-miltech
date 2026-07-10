#pragma once

#include "ballistic_app/interfaces/IBallisticSolver.h"
#include "ballistic_app/interfaces/ITargetProvider.h"
#include "ballistic_app/interfaces/IConfigLoader.h"
#include "ballistic_app/interfaces/ISimulationExporter.h"
#include <string>

namespace BallisticApp {

enum class SolverType { ANALYTICAL };
enum class TargetProviderType { JSON };
enum class ConfigLoaderType { FILE };
enum class ExporterType { JSON };

class ComponentFactory {
public:
  ComponentFactory() = delete;

  static IBallisticSolver* createSolver(SolverType type);
  static ITargetProvider* createProvider(TargetProviderType type, const std::string& param);
  static IConfigLoader* createLoader(ConfigLoaderType type);
  static ISimulationExporter* createExporter(ExporterType type, const std::string& param);
};

}  // namespace BallisticApp
