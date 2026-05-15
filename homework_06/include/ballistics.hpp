#pragma once

#include <string>
#include <array>

// ------------------------------------------------------------
// Константи та структури для балістики
// ------------------------------------------------------------
const float g_gravity = 9.81f;
const int BOMB_COUNT = 5;  // Кількість типів боєприпасів у каталозі

struct AmmoParams {
  std::string_view name;
  float mass;  // маса (кг)
  float drag;  // коефіцієнт опору
  float lift;  // коефіцієнт підйому
};

// Каталог боєприпасів
const std::array<AmmoParams, BOMB_COUNT> BOMB_CATALOG = {AmmoParams{"VOG-17", 0.35f, 0.07f, 0.0f},
                                                         AmmoParams{"M67", 0.6f, 0.10f, 0.0f},
                                                         AmmoParams{"RKG-3", 1.2f, 0.10f, 0.0f},
                                                         AmmoParams{"GLIDING-VOG", 0.45f, 0.10f, 1.0f},
                                                         AmmoParams{"GLIDING-RKG", 1.4f, 0.10f, 1.0f}};

// Структура для вхідних даних
struct BallisticsInput {
  float droneX, droneY, droneZ;
  float targetX, targetY;
  float attackSpeed;
  float accelerationPath;
  std::string ammoName;
};

// Структура для результатів розрахунку
struct DropSolution {
  float intermediateX = 0.0f;
  float intermediateY = 0.0f;
  float fireX = 0.0f;
  float fireY = 0.0f;
  bool hasIntermediate = false;
};

// ----------------------------------------------
// Зчитування вхідних даних з файлу
// ----------------------------------------------
BallisticsInput readInputData(const std::string& filename);

// ------------------------------------------------------------
// Пошук параметрів бомби у каталозі
// ------------------------------------------------------------
AmmoParams getAmmoParams(std::string_view name);

// ------------------------------------------------------------
// Балістика: час польоту (метод Кардано)
// ------------------------------------------------------------
float calcTimeOfFall(float z0, float v0, float m, float d, float l);

// ------------------------------------------------------------
// Балістика: горизонтальна дистанція (степеневий ряд до t^5)
// ------------------------------------------------------------
float calcHDistance(float t, float V0, float m, float d, float l);

// ------------------------------------------------------------
// Розрахунок балістики
// ------------------------------------------------------------
DropSolution computeDropSolution(const BallisticsInput& input);