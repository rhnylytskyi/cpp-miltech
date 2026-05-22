#pragma once

namespace BallisticApp {
class IBallisticSolver;
class ITargetProvider;
class IConfigLoader;
}  // namespace BallisticApp

namespace BallisticApp {

enum class SolverType { ANALYTICAL };
enum class TargetProviderType { JSON };
enum class ConfigLoaderType { FILE };

// Єдина фабрика для створення компонентів системи балістики
class Factory {
public:
  // Забороняємо створювати екземпляри фабрики, оскільки всі методи статичні
  Factory() = delete;

  static IBallisticSolver* createSolver(SolverType type);
  static ITargetProvider* createProvider(TargetProviderType type, const char* param);
  static IConfigLoader* createLoader(ConfigLoaderType type);
};

}  // namespace BallisticApp
