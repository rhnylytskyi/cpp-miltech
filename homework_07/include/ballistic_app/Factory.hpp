#pragma once

#include "ballistic_app/interfaces/IBallisticSolver.hpp"
#include "ballistic_app/interfaces/ITargetProvider.hpp"
#include "ballistic_app/interfaces/IConfigLoader.hpp"

namespace BallisticApp {

enum class SolverType { ANALYTICAL };
enum class TargetProviderType { JSON };
enum class ConfigLoaderType { FILE };

IBallisticSolver* createSolver(SolverType type);
ITargetProvider* createProvider(TargetProviderType type, const char* param);
IConfigLoader* createLoader(ConfigLoaderType type);

}  // namespace BallisticApp
