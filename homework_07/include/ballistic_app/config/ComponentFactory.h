#pragma once

namespace BallisticApp {
class IBallisticSolver;
class ITargetProvider;
class IConfigLoader;
class ISimulationExporter;
}  // namespace BallisticApp

namespace BallisticApp {

enum class SolverType { ANALYTICAL };
enum class TargetProviderType { JSON };
enum class ConfigLoaderType { FILE };
enum class ExporterType { JSON };

// Єдина фабрика для створення компонентів системи балістики
class ComponentFactory {
public:
  // Забороняємо створювати екземпляри фабрики, оскільки всі методи статичні
  ComponentFactory() = delete;

  static IBallisticSolver* createSolver(SolverType type);
  static ITargetProvider* createProvider(TargetProviderType type, const char* param);
  static IConfigLoader* createLoader(ConfigLoaderType type);
  static ISimulationExporter* createExporter(ExporterType type, const char* param);
};

}  // namespace BallisticApp
