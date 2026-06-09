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
 * @param altitude Висота скидання (має бути > 0), [метру].
 * @param speed Швидкість атаки дрона, [м/с].
 * @param mass  Маса боєприпасу, [кг].
 * @param drag  Коефіцієнт лобового опору повітря.
 * @param lift  Коефіцієнт підйомної сили (планування).
 *
 * @return float Розрахований час польоту в секундах.
 * @throws std::runtime_error Якщо початкова висота z0 <= 0.
 */
float calcTimeOfFall(const BallisticsArgs& args)
{
  float altitude = args.altitude;
  float speed = args.speed;
  float mass = args.mass;
  float drag = args.drag;
  float lift = args.lift;

  if (altitude <= 0) {
    throw std::runtime_error("Initial height must be greater than zero.");
  }

  const float a_coeff = drag * g_gravity * mass - 2.0F * drag * drag * lift * speed;
  const float b_coeff = -3.0F * g_gravity * mass * mass + 3.0F * drag * lift * mass * speed;
  const float c_coeff = 6.0F * mass * mass * altitude;

  if (std::fabs(a_coeff) < std::numeric_limits<float>::epsilon()) {
    return std::sqrt(2.0F * altitude / g_gravity);
  }

  const float p_coeff = -b_coeff * b_coeff / (3.0F * a_coeff * a_coeff);
  const float q_coeff = (2.0F * b_coeff * b_coeff * b_coeff) / (27.0F * a_coeff * a_coeff * a_coeff) + c_coeff / a_coeff;

  if (p_coeff >= 0.0F) {
    return std::sqrt(2.0F * altitude / g_gravity);
  }

  const float arg = 3.0F * q_coeff / (2.0F * p_coeff) * std::sqrt(-3.0F / p_coeff);
  if (std::fabs(arg) > 1.0F) {
    return std::sqrt(2.0F * altitude / g_gravity);
  }

  const float phi = std::acos(arg);

  const float time =
    2.0F * std::sqrt(-p_coeff / 3.0F) * std::cos((phi + 4.0F * std::numbers::pi_v<float>) / 3.0F) - b_coeff / (3.0F * a_coeff);

  return time > 0.0F ? time : std::sqrt(2.0F * altitude / g_gravity);
}

/**
 * @brief Розраховує горизонтальну дистанцію польоту боєприпасу.
 *
 * @param time  Час польоту, [с].
 * @param speed Початкова швидкість атаки дрона, [м/с].
 * @param mass  Маса боєприпасу, [кг].
 * @param drag  Коефіцієнт лобового опору повітря.
 * @param lift  Коефіцієнт підйомної сили (планування).
 *
 * @return float Розрахована горизонтальна дистанція, [м].
 */
float calcHDistance(float time, float speed, float mass, float drag, float lift)
{
  const float lift_2 = lift * lift;
  const float lift_4 = lift_2 * lift_2;

  // Оптимальне обчислення степенів часу без використання важкої функції std::pow
  const float time_2 = time * time;
  const float time_3 = time_2 * time;
  const float time_4 = time_3 * time;
  const float time_5 = time_4 * time;

  const float drag_2 = drag * drag;
  const float drag_3 = drag_2 * drag;
  const float drag_4 = drag_3 * drag;

  // Оптимізація: std::pow(m, 3) замінено на чисте множення m * m * m (для float це значно швидше)
  const float mass_2 = mass * mass;
  const float mass_3 = mass_2 * mass;
  const float mass_4 = mass_3 * mass;

  // Знаменник для третього доданку (1.0f + lift_2)^2
  const float denomFactor = (1.0F + lift_2) * (1.0F + lift_2);

  const float hDist = speed * time - (drag * time_2 * speed) / (2.0F * mass) +
                      (time_3 * (6.0F * drag * g_gravity * lift * mass - 6.0F * drag_2 * (-1.0F + lift_2) * speed)) / (36.0F * mass_2) +
                      (time_4 * (-6.0F * drag_2 * g_gravity * lift * (1.0F + lift_2 + lift_4) * mass +
                                 3.0F * drag_3 * lift_2 * (1.0F + lift_2) * speed + 6.0F * drag_3 * lift_4 * (1.0F + lift_2) * speed)) /
                        (36.0F * denomFactor * mass_3) +
                      (time_5 * (3.0F * drag_3 * g_gravity * lift_2 * lift * mass - 3.0F * drag_4 * lift_2 * (1.0F + lift_2) * speed)) /
                        (36.0F * (1.0F + lift_2) * mass_4);

  return hDist;
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
  float mass = ammoParams.mass;
  float drag = ammoParams.drag;
  float lift = ammoParams.lift;
  BallisticsArgs args{input.droneZ, input.attackSpeed, mass, drag, lift};

  float flightTime = calcTimeOfFall(args);
  float hDist = calcHDistance(flightTime, args.speed, args.mass, args.drag, args.lift);

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
