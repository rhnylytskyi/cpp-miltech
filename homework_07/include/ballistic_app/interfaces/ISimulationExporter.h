#pragma once

#include "ballistic_app/Types.h"

namespace BallisticApp {

class ISimulationExporter {
public:
  virtual ~ISimulationExporter() = default;

  /**
   * @brief Чисто віртуальний метод для експорту результатів симуляції.
   *
   * @param history Масив кроків симуляції.
   * @param totalSteps Загальна кількість кроків.
   * @throws std::runtime_error Якщо передано некоректні дані або виникла помилка запису у файл.
   */
  virtual void exportSimulation(const SimStep* history, int totalSteps) const = 0;
};

}  // namespace BallisticApp
