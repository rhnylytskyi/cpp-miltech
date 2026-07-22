#pragma once

namespace BallisticApp {

struct DroneConfig {
  float attackSpeed{0.0f};
  float accelerationPath{0.0f};
  float angularSpeed{0.0f};
  float turnThreshold{0.0f};
  float timeStep{0.0f};
  float timeScale{1.0f};

  // Додаткове поле, яке передається з AmmoCfg для розрахунку зони скиду
  float hitRadius{0.0f};
};

}  // namespace BallisticApp