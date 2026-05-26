#pragma once

#include <string>
#include <array>
#include <numbers>

// ------------------------------------------------------------
// Константи та структури для балістики
// ------------------------------------------------------------
const float g_gravity = 9.81F;
const int BOMB_COUNT = 5;                      // Кількість типів боєприпасів у каталозі
const float pi_f = std::numbers::pi_v<float>;  // Має тип float (запобігає narrowing conversions)
const double pi = std::numbers::pi;            // Має тип double

struct AmmoParams {
  std::string_view name;
  float mass;  // маса (кг)
  float drag;  // коефіцієнт опору
  float lift;  // коефіцієнт підйому
};

// Каталог боєприпасів
const std::array<AmmoParams, BOMB_COUNT> BOMB_CATALOG = {AmmoParams{"VOG-17", 0.35F, 0.07F, 0.0F},
                                                         AmmoParams{"M67", 0.6F, 0.10F, 0.0F},
                                                         AmmoParams{"RKG-3", 1.2F, 0.10F, 0.0F},
                                                         AmmoParams{"GLIDING-VOG", 0.45F, 0.10F, 1.0F},
                                                         AmmoParams{"GLIDING-RKG", 1.4F, 0.10F, 1.0F}};

// Структура для вхідних даних
struct BallisticsInput {
  float droneX{}, droneY{}, droneZ{};
  float targetX{}, targetY{};
  float attackSpeed{};
  float accelerationPath{};
  std::string ammoName;
};

// Структура для результатів розрахунку
struct DropSolution {
  float intermediateX = 0.0F;
  float intermediateY = 0.0F;
  float fireX = 0.0F;
  float fireY = 0.0F;
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