#define _USE_MATH_DEFINES

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
BallisticsInput readInputData(const std::string& filename) {
    std::ifstream fin(filename);
    if (!fin.is_open()) {
        throw std::runtime_error("Cannot open input file.");
    }

    BallisticsInput input;
    if (!(fin >> input.droneX >> input.droneY >> input.droneZ 
            >> input.targetX >> input.targetY 
            >> input.attackSpeed 
            >> input.accelerationPath 
            >> input.ammoName)) {
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
AmmoParams getAmmoParams(std::string_view name) {
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
 * @param z0 Початкова висота скидання (має бути > 0), [метру].
 * @param v0 Початкова швидкість атаки дрона, [м/с].
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

    if (std::fabs(a) < 1e-12f)
        return std::sqrt(2.0f * z0 / g_gravity);

    float p = -b * b / (3 * a * a);
    float q = (2 * b * b * b) / (27 * a * a * a) + c / a;

    if (p >= 0)
        return std::sqrt(2.0f * z0 / g_gravity);

    float arg = 3 * q / (2 * p) * std::sqrt(-3 / p);
    if (std::fabs(arg) > 1)
        return std::sqrt(2.0f * z0 / g_gravity);

    float phi = std::acos(arg);
    float t = 2 * std::sqrt(-p / 3) * std::cos((phi + 4 * (float)M_PI) / 3) - b / (3 * a);
    return t > 0 ? t : std::sqrt(2.0f * z0 / g_gravity);
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

    float h = t * V0
        - (d * std::pow(t, 2) * V0) / (2 * m)
        + (std::pow(t, 3) * (6 * d * g_gravity * l * m - 8 * std::pow(d, 2) * (-1 + l2) * V0)) / (36 * std::pow(m, 2))
        + (std::pow(t, 4) * (-6 * std::pow(d, 2) * g_gravity * l * (1 + l2 + l4) * m
            + 3 * std::pow(d, 3) * l2 * (1 + l2) * V0
            + 6 * std::pow(d, 3) * l4 * (1 + l2) * V0))
        / (36 * std::pow(1 + l2, 2) * std::pow(m, 3))
        + (std::pow(t, 5) * (3 * std::pow(d, 3) * g_gravity * std::pow(l, 3) * m
            - 3 * std::pow(d, 4) * l2 * (1 + l2) * V0))
        / (36 * (1 + l2) * std::pow(m, 4));

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

    float distanceToTarget = std::sqrt(std::pow(input.targetX - droneX, 2) + std::pow(input.targetY - droneY, 2));

    DropSolution solution;
    solution.hasIntermediate = hDist + input.accelerationPath > distanceToTarget;

    if (solution.hasIntermediate)
    {
        if (std::fabs(distanceToTarget) < 1e-6)
        {
            droneX = input.targetX - (hDist + input.accelerationPath);
            droneY = input.targetY;
            distanceToTarget = hDist + input.accelerationPath;
        }
        else
        {
            droneX = input.targetX - (input.targetX - droneX) * (hDist + input.accelerationPath) / distanceToTarget;
            droneY = input.targetY - (input.targetY - droneY) * (hDist + input.accelerationPath) / distanceToTarget;
            distanceToTarget = std::sqrt(std::pow(input.targetX - droneX, 2) + std::pow(input.targetY - droneY, 2));
        }
        solution.intermediateX = droneX;
        solution.intermediateY = droneY;
    }

    float ratio = (distanceToTarget - hDist) / distanceToTarget;

    solution.fireX = droneX + (input.targetX - droneX) * ratio;
    solution.fireY = droneY + (input.targetY - droneY) * ratio;
    
    return solution;
}
