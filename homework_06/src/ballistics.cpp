#define _USE_MATH_DEFINES

#include "ballistics.hpp"

#include <cmath>

// ------------------------------------------------------------
// Балістика з ДЗ 1: час польоту (метод Кардано)
// ------------------------------------------------------------
float calcTimeOfFall(float z0, float v0, float m, float d, float l)
{
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

// ------------------------------------------------------------
// Балістика з ДЗ 1: горизонтальна дистанція (степеневий ряд до t^5)
// ------------------------------------------------------------
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

void getIntermediateAndDropPoint(float currentX, float currentY, float targetX, float targetY, float hDistance, float accelerationPath, float& outIntermediateX, float& outIntermediateY,
                               float& outFireX, float& outFireY, bool& outHasIntermediate)
{
    float distanceToTarget = std::sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2));

    outHasIntermediate = hDistance + accelerationPath > distanceToTarget;

    if (outHasIntermediate)
    {
        if (fabs(distanceToTarget) < 1e-6)
        {
            currentX = targetX - (hDistance + accelerationPath);
            currentY = targetY;
            distanceToTarget = hDistance + accelerationPath;
        }
        else
        {
            currentX = targetX - (targetX - currentX) * (hDistance + accelerationPath) / distanceToTarget;
            currentY = targetY - (targetY - currentY) * (hDistance + accelerationPath) / distanceToTarget;
            distanceToTarget = std::sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2));
        }
        outIntermediateX = currentX;
        outIntermediateY = currentY;
    }

    float ratio = (distanceToTarget - hDistance) / distanceToTarget;

    outFireX = currentX + (targetX - currentX) * ratio;
    outFireY = currentY + (targetY - currentY) * ratio;
}