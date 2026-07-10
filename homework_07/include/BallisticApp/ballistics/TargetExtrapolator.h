#pragma once

#include "BallisticApp/types/Coord.h"

namespace BallisticApp {

class ITargetProvider;

class TargetExtrapolator {
public:
  TargetExtrapolator(const ITargetProvider& provider, float targetTimeStep);

  /**
   * @brief Лінійна інтерполяція позиції цілі для заданого моменту часу.
   * @param targetIdx Індекс цілі.
   * @param t Поточний момент часу симуляції (секунди).
   * @return Coord Координати цілі в момент часу t.
   */
  Coord interpolate(int targetIdx, float t) const;

  /**
   * @brief Лінійна екстраполяція позиції цілі на основі поточної швидкості.
   * @param targetIdx Індекс цілі.
   * @param time Поточний час симуляції (секунди).
   * @param dt Інтервал часу для прогнозу вперед (секунди).
   * @return Coord Прогнозовані координати цілі через час dt.
   */
  Coord extrapolate(int targetIdx, float time, float dt) const;

private:
  const ITargetProvider& m_provider;
  float m_targetTimeStep;
};

}  // namespace BallisticApp
