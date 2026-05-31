#include "BallisticApp/ComponentFactory.h"
#include "BallisticApp/config/FileConfigLoader.h"
#include "BallisticApp/providers/JsonTargetProvider.h"
#include "BallisticApp/solvers/AnalyticalBallisticSolver.h"
#include "BallisticApp/exporters/JsonExporter.h"
#include "BallisticApp/types/DroneStateType.h"
#include "BallisticApp/interfaces/IDroneState.h"
#include "BallisticApp/states/StateStopped.h"
#include "BallisticApp/states/StateAccelerating.h"
#include "BallisticApp/states/StateDecelerating.h"
#include "BallisticApp/states/StateTurning.h"
#include "BallisticApp/states/StateMoving.h"
#include <array>
#include <memory>
#include <filesystem>

namespace BallisticApp {

std::unique_ptr<IBallisticSolver> ComponentFactory::createSolver(SolverType type)
{
  switch (type) {
    case SolverType::ANALYTICAL:
      return std::make_unique<AnalyticalBallisticSolver>();
    default:
      return nullptr;
  }
}

std::unique_ptr<ITargetProvider> ComponentFactory::createProvider(TargetProviderType type, const std::filesystem::path& param)
{
  switch (type) {
    case TargetProviderType::JSON:
      return std::make_unique<JsonTargetProvider>(param);
    default:
      return nullptr;
  }
}

std::unique_ptr<IConfigLoader> ComponentFactory::createLoader(ConfigLoaderType type)
{
  switch (type) {
    case ConfigLoaderType::FILE:
      return std::make_unique<FileConfigLoader>();
    default:
      return nullptr;
  }
}

std::unique_ptr<ISimulationExporter> ComponentFactory::createExporter(ExporterType type, const std::filesystem::path& param)
{
  switch (type) {
    case ExporterType::JSON:
      return std::make_unique<JsonExporter>(param);
    default:
      return nullptr;
  }
}

IDroneState* ComponentFactory::getState(DroneStateType type)
{
  static const std::array<std::unique_ptr<IDroneState>, 5> s_registry = []() {
    std::array<std::unique_ptr<IDroneState>, 5> pool;
    pool[static_cast<size_t>(DroneStateType::STOPPED)] = std::make_unique<StateStopped>();
    pool[static_cast<size_t>(DroneStateType::ACCELERATING)] = std::make_unique<StateAccelerating>();
    pool[static_cast<size_t>(DroneStateType::DECELERATING)] = std::make_unique<StateDecelerating>();
    pool[static_cast<size_t>(DroneStateType::TURNING)] = std::make_unique<StateTurning>();
    pool[static_cast<size_t>(DroneStateType::MOVING)] = std::make_unique<StateMoving>();
    return pool;
  }();

  return s_registry[static_cast<size_t>(type)].get();
}
}  // namespace BallisticApp
