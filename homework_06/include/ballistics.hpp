#pragma once

#include <string_view>

// ==========================================================
// Константи
// ==========================================================
inline constexpr int BOMB_COUNT = 5;
inline constexpr float g_gravity = 9.81f;

// ==========================================================
// Параметри боєприпасів (масиви)
// ==========================================================
inline constexpr char  bombNames[BOMB_COUNT][15] = {"VOG-17", "M67", "RKG-3", "GLIDING-VOG", "GLIDING-RKG"};
inline constexpr float bombM[BOMB_COUNT]         = {0.35f,   0.6f,  1.2f,    0.45f,         1.4f};
inline constexpr float bombD[BOMB_COUNT]         = {0.07f,   0.10f, 0.10f,   0.10f,         0.10f};
inline constexpr float bombL[BOMB_COUNT]         = {0.0f,    0.0f,  0.0f,    1.0f,          1.0f};

// --- Пошук боєприпасу за назвою (цикл for) ---
inline int findBombIndexByName(std::string_view name)
{
    for (int i = 0; i < BOMB_COUNT; i++)
    {
        // Сучасне порівняння рядків без std::strcmp
        if (name == bombNames[i])
        {
            return i;
        }
    }
    return -1;
}

// ------------------------------------------------------------
// Балістика з ДЗ 1: час польоту (метод Кардано)
// ------------------------------------------------------------
float calcTimeOfFall(float z0, float v0, float m, float d, float l);

// ------------------------------------------------------------
// Балістика з ДЗ 1: горизонтальна дистанція (степеневий ряд до t^5)
// ------------------------------------------------------------
float calcHDistance(float t, float V0, float m, float d, float l);

void getIntermediateAndDropPoint(float currentX, float currentY, float targetX, float targetY, float hDistance, float accelerationPath, float& outIntermediateX, float& outIntermediateY,
                               float& outFireX, float& outFireY, bool& outHasIntermediate);