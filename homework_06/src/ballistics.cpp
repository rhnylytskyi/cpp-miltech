#include "ballistics.hpp"

#include <cmath>
#include <stdexcept>
#include <string_view>
#include <fstream>

/**
 * @brief Зчитує вхідні дані з файлу.
 * @param filename Ім'я файлу.
 * @return BallisticsInput Вхідні параметри балістики.
 */
BallisticsInput readInputData(const std::string& filename)
{
  std::ifstream fin(filename);
  if (!fin.is_open()) {
    throw std::runtime_error("Cannot open input file.");
  }

  BallisticsInput input;
  if (!(fin >> input.droneX >> input.droneY >> input.droneZ >> input.targetX >> input.targetY >> input.attackSpeed >>
        input.accelerationPath >> input.ammoName)) {
    throw std::runtime_error("Invalid data format in input file.");
  }

  fin.close();
  return input;
}

/**
 * @brief Пошук параметрів боєприпасу у каталозі за назвою.
 * @param name Назва боєприпасу.
 * @return AmmoParams Параметри боєприпасу.
 */
AmmoParams getAmmoParams(std::string_view name)
{
  for (const auto& ammo : BOMB_CATALOG) {
    if (ammo.name == name) {
      return ammo;
    }
  }
  throw std::runtime_error("Unknown ammo name: " + std::string(name));
}

/**
 * @brief Розраховує час падіння боєприпасу за методом Кардано.
 *
 * @param z0 Висота скидання (має бути > 0), [метру].
 * @param v0 Швидкість атаки дрона, [м/с].
 * @param m  Маса боєприпасу, [кг].
 * @param d  Коефіцієнт лобового опору повітря.
 * @param l  Коефіцієнт підйомної сили (планування).
 *
 * @return float Розрахований час польоту в секундах.
 * @throws std::runtime_error Якщо початкова висота z0 <= 0.
 */
float calcTimeOfFall(float z0, float v0, float m, float d, float l)
{
  if (z0 <= 0) {
    throw std::runtime_error("Initial height must be greater than zero.");
  }

  float a = d * g_gravity * m - 2 * d * l * v0;
  float b = -3 * g_gravity * m + 3 * d * l * v0;
  float c = 6 * m * z0;

  if (std::fabs(a) < 1e-12F) {
    return std::sqrt(2.0F * z0 / g_gravity);
  }

  float p = -b * b / (3 * a * a);
  float q = (2 * b * b * b) / (27 * a * a * a) + c / a;

  if (p >= 0) {
    return std::sqrt(2.0F * z0 / g_gravity);
  }

  float arg = 3 * q / (2 * p) * std::sqrt(-3 / p);
  if (std::fabs(arg) > 1) {
    return std::sqrt(2.0F * z0 / g_gravity);
  }

  float phi = std::acos(arg);
  float t = 2 * std::sqrt(-p / 3) * std::cos((phi + 4 * pi_f) / 3) - b / (3 * a);
  return t > 0 ? t : std::sqrt(2.0F * z0 / g_gravity);
}

/**
 * @brief Розраховує горизонтальну дистанцію польоту боєприпасу.
 *
 * @param t  Час польоту, [с].
 * @param V0 Початкова швидкість атаки дрона, [м/с].
 * @param m  Маса боєприпасу, [кг].
 * @param d  Коефіцієнт лобового опору повітря.
 * @param l  Коефіцієнт підйомної сили (планування).
 *
 * @return float Розрахована горизонтальна дистанція, [м].
 */
float calcHDistance(float t, float V0, float m, float d, float l)
{
  float l2 = l * l;
  float l4 = l2 * l2;
  float d2 = d * d;
  float m2 = m * m;
  float t2 = t * t;
  float t4 = t2 * t2;

  float l_factor = 1.0F + l2;
  float m4 = m2 * m2;
  float l3 = l2 * l;

  // Попереднє обчислення О Б Е Р Н Е Н И Х знаменників (1 / знаменник)
  float inv_denom1 = 1.0F / (2.0F * m);
  float inv_denom2 = 1.0F / (36.0F * m2);
  float inv_denom3 = 1.0F / (36.0F * l_factor * l_factor * m2 * m);
  float inv_denom4 = 1.0F / (36.0F * l_factor * m4);

  // Тепер замість важкого ділення (/) всюди використовується швидке множення (*)
  float h =
    t * V0 - (d * t2 * V0) * inv_denom1 + (t2 * t * (6.0F * d * g_gravity * l * m - 8.0F * d2 * (-1.0F + l2) * V0)) * inv_denom2 +
    (t4 * (-6.0F * d2 * g_gravity * l * (1.0F + l2 + l4) * m + 3.0F * d2 * l2 * (1.0F + l2) * V0 + 6.0F * d2 * l4 * (1.0F + l2) * V0)) *
      inv_denom3 +
    (t4 * t * (3.0F * d2 * g_gravity * l3 * m - 3.0F * d2 * l2 * (1.0F + l2) * V0)) * inv_denom4;

  return h;
}

/**
 * @brief Розрахунок балістики для заданих вхідних параметрів.
 *
 * @param input Вхідні параметри балістики.
 * @return DropSolution Результат обчислення.
 */
DropSolution computeDropSolution(const BallisticsInput& input)
{
  AmmoParams ammoParams = getAmmoParams(input.ammoName);
  float m = ammoParams.mass;
  float d = ammoParams.drag;
  float l = ammoParams.lift;

  float flightTime = calcTimeOfFall(input.droneZ, input.attackSpeed, m, d, l);
  float hDist = calcHDistance(flightTime, input.attackSpeed, m, d, l);

  // Створюємо локальні копії координат дрона, щоб НЕ змінювати оригінальний input
  float droneX = input.droneX;
  float droneY = input.droneY;
  float distanceToTarget = std::hypot(input.targetX - droneX, input.targetY - droneY);

  DropSolution solution;
  solution.hasIntermediate = hDist + input.accelerationPath > distanceToTarget;

  if (solution.hasIntermediate) {
    if (std::fabs(distanceToTarget) < 1e-6) {
      droneX = input.targetX - (hDist + input.accelerationPath);
      droneY = input.targetY;
      distanceToTarget = hDist + input.accelerationPath;
    }
    else {
      droneX = input.targetX - (input.targetX - droneX) * (hDist + input.accelerationPath) / distanceToTarget;
      droneY = input.targetY - (input.targetY - droneY) * (hDist + input.accelerationPath) / distanceToTarget;
      distanceToTarget = std::hypot(input.targetX - droneX, input.targetY - droneY);
    }
    solution.intermediateX = droneX;
    solution.intermediateY = droneY;
  }

  float ratio = (distanceToTarget - hDist) / distanceToTarget;

  solution.fireX = droneX + (input.targetX - droneX) * ratio;
  solution.fireY = droneY + (input.targetY - droneY) * ratio;

  return solution;
}
