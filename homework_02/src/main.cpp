#define _USE_MATH_DEFINES
#include <cmath>
#include <fstream>
#include <iostream>
#include <cstring>

// ==========================================================
// Стани дрона (enum)
// ==========================================================
enum DroneState
{
    STOPPED      = 0,
    ACCELERATING = 1,
    DECELERATING = 2,
    TURNING      = 3,
    MOVING       = 4
};

// ==========================================================
// Константи
// ==========================================================
const int   BOMB_COUNT = 5;
const int   TARGET_COUNT = 5;
const int   TARGET_ARRAY_SIZE = 60;
const float g_gravity = 9.81f;
const int   MAX_STEPS = 10000;
const int   NUM_TIME_APPROXIMATION_STEPS = 10;

#define STOP_WHEN_TURNING

// ==========================================================
// Параметри боєприпасів (масиви)
// ==========================================================
char  bombNames[BOMB_COUNT][15] = {"VOG-17", "M67", "RKG-3", "GLIDING-VOG", "GLIDING-RKG"};
float bombM[BOMB_COUNT]         = {0.35f,   0.6f,  1.2f,    0.45f,         1.4f};
float bombD[BOMB_COUNT]         = {0.07f,   0.10f, 0.10f,   0.10f,         0.10f};
float bombL[BOMB_COUNT]         = {0.0f,    0.0f,  0.0f,    1.0f,          1.0f};

// ==========================================================
// Масиви координат цілей
// ==========================================================
float targetXInTime[TARGET_COUNT][TARGET_ARRAY_SIZE];
float targetYInTime[TARGET_COUNT][TARGET_ARRAY_SIZE];

// Вихідні масиви
// ------------------------------------------------------------
float outX[MAX_STEPS];
float outY[MAX_STEPS];
float outDir[MAX_STEPS];
int outState[MAX_STEPS];
int outTarget[MAX_STEPS];
float outFireX[MAX_STEPS];
float outFireY[MAX_STEPS];
float outPredTgtX[MAX_STEPS];
float outPredTgtY[MAX_STEPS];

// ------------------------------------------------------------
// Інтерполяція позиції цілі
// ------------------------------------------------------------
void interpolateTarget(int targetIdx, float t, float arrayTimeStep,
                      float& outTx, float& outTy) 
{
    int idx = (int)std::floor(t / arrayTimeStep) % TARGET_ARRAY_SIZE;
    int next = (idx + 1) % TARGET_ARRAY_SIZE;
    float frac = (t / arrayTimeStep) - std::floor(t / arrayTimeStep);

    outTx = targetXInTime[targetIdx][idx] 
            + (targetXInTime[targetIdx][next] - targetXInTime[targetIdx][idx]) * frac;
    outTy = targetYInTime[targetIdx][idx] 
            + (targetYInTime[targetIdx][next] - targetYInTime[targetIdx][idx]) * frac;
}

// Лінійна екстраполяція: знаємо лише поточний відрізок (idx, idx+1),
// прогнозуємо що ціль продовжує рухатись з тією ж швидкістю і напрямком.
// currentTime - поточний час, dt - на скільки секунд вперед прогнозуємо.
void extrapolateTarget(int targetIdx, float currentTime, float dt,
                      float arrayTimeStep, float& outTx, float& outTy)
{
    int idx = (int)std::floor(currentTime / arrayTimeStep) % TARGET_ARRAY_SIZE;
    int next = (idx + 1) % TARGET_ARRAY_SIZE;

    // Швидкість цілі на поточному відрізку
    float vx = (targetXInTime[targetIdx][next] - targetXInTime[targetIdx][idx]) / arrayTimeStep;
    float vy = (targetYInTime[targetIdx][next] - targetYInTime[targetIdx][idx]) / arrayTimeStep;

    // Поточна позиція (інтерполяція)
    float curX, curY;
    interpolateTarget(targetIdx, currentTime, arrayTimeStep, curX, curY);

    // Екстраполяція
    outTx = curX + vx * dt;
    outTy = curY + vy * dt;
}

// ------------------------------------------------------------
// Нормалізація кута до [-PI, PI]
// ------------------------------------------------------------
float normalizeAngle(float a)
{
    while (a > M_PI) a -= 2.0f * (float)M_PI;
    while (a < -M_PI) a += 2.0f * (float)M_PI;
    return a;
}

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

bool needStop(float desiredDir, float currentDir, float angleThreshold)
{
#ifdef STOP_WHEN_TURNING
    float deltaAngle = normalizeAngle(desiredDir - currentDir);

    return std::fabs(deltaAngle) > angleThreshold;
#else
    return false;
#endif
}

float accel(float speed, float accelPath)
{
    return speed * speed / (2.0f * accelPath);
}

float accelTime(float speed, float acc)
{
    return speed / acc;
}

float accelTimeFromDist(float dist, float acc)
{
    return std::sqrt(2.0f * dist / acc);
}

float accelPathFromSpeed(float speed, float acc)
{
    return (speed * speed) / (2.0f * acc);
}

float distFromSpeedAndAccel(float speed, float acc)
{
    return (speed * speed) / (2.0f * acc);
}

// ------------------------------------------------------------
// Розрахунок часу польоту до заданої координати
// ------------------------------------------------------------
float calcTimeOfFlight(float currentX, float currentY, float targetX, float targetY, float currentDir, float angleThreshold, float angularSpeed, float currentSpeed, float maxSpeed, float accelPath, bool needToStopAtTarget)
{
    float totalTimeToPoint = 0.0f;
    float desiredDir = std::atan2(targetY - currentY, targetX - currentX);
    float a = accel(maxSpeed, accelPath);

    bool ns = needStop(desiredDir, currentDir, angleThreshold);

    if (ns)
    {
        float pathToStop = (currentSpeed * currentSpeed) / (2.0f * a);

        currentX += std::cos(currentDir) * pathToStop;
        currentY += std::sin(currentDir) * pathToStop;

        totalTimeToPoint += accelTime(currentSpeed, a);
        totalTimeToPoint += std::fabs(normalizeAngle(desiredDir - currentDir)) / angularSpeed;

        currentSpeed = 0;
        currentDir = desiredDir;
    }

    float distanceToTarget = std::hypot(targetX - currentX, targetY - currentY);
    float decceleratePath = std::min(needToStopAtTarget ? accelPathFromSpeed(maxSpeed, a) : 0, distanceToTarget);
    distanceToTarget -= decceleratePath;
    totalTimeToPoint += accelTimeFromDist(decceleratePath, a);

    float accelDist = std::min(accelPathFromSpeed(maxSpeed, a) - accelPathFromSpeed(currentSpeed, a), distanceToTarget);
    totalTimeToPoint += accelTimeFromDist(accelDist, a);
    distanceToTarget -= accelDist;
    totalTimeToPoint += distanceToTarget / maxSpeed;

    return totalTimeToPoint;
}

// ------------------------------------------------------------
// MAIN
// ------------------------------------------------------------
int main(int argc, char** argv)
{
    // --- Читання input.txt ---
    float xd, yd, zd;
    float initialDir;
    float attackSpeed;
    float accelerationPath;
    char ammo_name[15] = "";
    float arrayTimeStep;
    float simTimeStep;
    float hitRadius;
    float angularSpeed;
    float turnThreshold;

    std::ifstream fin(DATA_PATH);
    if (!fin.is_open()) { std::cerr << "Cannot open input.txt" << std::endl; return 1; }
    fin >> xd >> yd >> zd;
    fin >> initialDir;
    fin >> attackSpeed;
    fin >> accelerationPath;
    fin >> ammo_name;
    fin >> arrayTimeStep;
    fin >> simTimeStep;
    fin >> hitRadius;
    fin >> angularSpeed;
    fin >> turnThreshold;
    fin.close();

    // --- Пошук боєприпасу за назвою (цикл for) ---
    int bombIdx = -1;
    for (int i = 0; i < BOMB_COUNT; i++)
    {
        if (std::strcmp(ammo_name, bombNames[i]) == 0)
        {
            bombIdx = i;
            break;
        }
    }

    if (bombIdx < 0) { std::cerr << "Unknown ammo: " << ammo_name << std::endl; return 1; }

    float m = bombM[bombIdx];
    float d = bombD[bombIdx];
    float l = bombL[bombIdx];

     // --- Читання targets.txt ---
    std::ifstream ftgt(TARGETS_PATH);
    if (!ftgt.is_open()) { std::cerr << "Cannot open targets.txt" << std::endl; return 1; }
    for (int i = 0; i < 5; i++)
    {
        for (int j = 0; j < 60; j++)
            ftgt >> targetXInTime[i][j];
    }
    for (int i = 0; i < 5; i++)
    {
        for (int j = 0; j < 60; j++)
            ftgt >> targetYInTime[i][j];
    }
    ftgt.close();

    // --- Параметри руху дрона ---
    float acceleration = attackSpeed * attackSpeed / (2.0f * accelerationPath);

    float droneX = xd;
    float droneY = yd;
    float direction = initialDir;
    float speed = 0.0f;
    DroneState state = STOPPED;

    float currentTime = 0.0f;
    int currentTarget = -1;
    float targetDir = initialDir;

#ifdef STOP_WHEN_TURNING
    float turnRemaining = 0.0f;
#endif

    int step = 0;

    // Попередній розрахунок балістичних констант (залежать лише від висоти та снаряду)
    float flightTime = calcTimeOfFall(zd, attackSpeed, m, d, l);
    float hDist = calcHDistance(flightTime, attackSpeed, m, d, l);

    // --- Основний цикл симуляції ---
    while (step < MAX_STEPS)
    {
        float bestTime = 1e9f;
        int bestTarget = 0;
        float bestPredX = droneX, bestPredY = droneY;
        float bestFireX = droneX, bestFireY = droneY;
        float desiredDirectionForBest = direction;
        float fx = 0, fy = 0;

        for (int targetId = 0; targetId < TARGET_COUNT; ++targetId)
        {
            float predX = 0, predY = 0;
            float totalTime = 0;
            bool hasIntermediate;

            for (int iter = 0; iter < NUM_TIME_APPROXIMATION_STEPS; iter++)
            {
                extrapolateTarget(targetId, currentTime, totalTime + flightTime, arrayTimeStep, predX, predY);

                float intermediateX, intermediateY;
                float fireX, fireY;
                getIntermediateAndDropPoint(droneX, droneY, predX, predY, hDist, accelerationPath, intermediateX, intermediateY, fireX, fireY, hasIntermediate);

                // Якщо дрон вже летить до поточної цілі - не відступаємо назад
                if (hasIntermediate && targetId == currentTarget && (state == MOVING || state == ACCELERATING))
                    hasIntermediate = false;

                totalTime = 0;
                if (hasIntermediate)
                {
                    totalTime = calcTimeOfFlight(droneX, droneY, intermediateX, intermediateY, direction, turnThreshold, angularSpeed, speed, attackSpeed, accelerationPath, true);
                    float directionToFire = std::atan2(fireY - intermediateY, fireX - intermediateX);
                    totalTime += std::fabs(normalizeAngle(directionToFire - direction)) / angularSpeed;
                    totalTime += calcTimeOfFlight(intermediateX, intermediateY, fireX, fireY, directionToFire, turnThreshold, angularSpeed, attackSpeed, attackSpeed, accelerationPath, false);
                    
                    fx = fireX;
                    fy = fireY;
                }
                else
                {
                    totalTime = calcTimeOfFlight(droneX, droneY, fireX, fireY, direction, turnThreshold, angularSpeed, speed, attackSpeed, accelerationPath, false);
                    fx = fireX;
                    fy = fireY;
                }
            }

            if (totalTime < bestTime)
            {
                bestTime = totalTime;
                bestTarget = targetId;
                bestPredX = predX;
                bestPredY = predY;
                bestFireX = fx;
                bestFireY = fy;
            }
        }

        currentTarget = bestTarget;

        // Бажаний напрямок на fire point найкращої цілі
        desiredDirectionForBest = std::atan2(bestFireY - droneY, bestFireX - droneX);

         // 4. Записати дані кроку у вихідні масиви
        outX[step]        = droneX;
        outY[step]        = droneY;
        outDir[step]      = direction;
        outState[step]    = (int)state;
        outTarget[step]   = currentTarget;
        outFireX[step]    = bestFireX;
        outFireY[step]    = bestFireY;
        outPredTgtX[step] = bestPredX;
        outPredTgtY[step] = bestPredY;

        float deltaPath = 0;
        float deltaAngle = normalizeAngle(desiredDirectionForBest - direction);

        // 6. Автомат станів (самоуправлінний, як у ДЗ3)
        switch (state)
        {
            case STOPPED:
            {
                if (std::fabs(deltaAngle) > turnThreshold)
                {
                    state = TURNING;
                }
                else
                {
                    direction = desiredDirectionForBest;
                    state = ACCELERATING;
                }
                break;
            }

            case ACCELERATING:
            {
                if (std::fabs(deltaAngle) > turnThreshold && speed > 0.01f)
                {
                    // Треба повернути - спочатку гальмуємо
                    state = DECELERATING;
                    float prevSpeed = speed;
                    speed -= acceleration * simTimeStep;
                    if (speed <= 0) { speed = 0; state = STOPPED; }
                    deltaPath = (prevSpeed + speed) / 2.0f * simTimeStep;
                }
                else
                {
                    // Малі поправки курсу на льоту
                    if (std::fabs(deltaAngle) <= turnThreshold)
                        direction = desiredDirectionForBest;

                    float prevSpeed = speed;
                    speed += acceleration * simTimeStep;
                    if (speed >= attackSpeed)
                    {
                        speed = attackSpeed;
                        state = MOVING;
                    }
                    deltaPath = (prevSpeed + speed) / 2.0f * simTimeStep;
                }
                break;
            }

            case DECELERATING:
            {
                float prevSpeed = speed;
                speed -= acceleration * simTimeStep;
                if (speed <= 0)
                {
                    speed = 0;
                    state = STOPPED;
                }
                deltaPath = (prevSpeed + speed) / 2.0f * simTimeStep;
                break;
            }

            case TURNING:
            {
                float da = normalizeAngle(desiredDirectionForBest - direction);
                if (std::fabs(da) <= angularSpeed * simTimeStep)
                {
                    direction = desiredDirectionForBest;
                    state = ACCELERATING;
                }
                else
                {
                    direction += (da > 0 ? 1.0f : -1.0f) * angularSpeed * simTimeStep;
                    direction = normalizeAngle(direction);
                }
                break;
            }

            case MOVING:
            {
                if (std::fabs(deltaAngle) > turnThreshold)
                {
                    state = DECELERATING;
                    float prevSpeed = speed;
                    speed -= acceleration * simTimeStep;
                    if (speed <= 0) { speed = 0; state = STOPPED; }
                    deltaPath = (prevSpeed + speed) / 2.0f * simTimeStep;
                }
                else
                {
                    // Малі поправки курсу на льоту
                    if (std::fabs(deltaAngle) <= turnThreshold)
                        direction = desiredDirectionForBest;
                    deltaPath = speed * simTimeStep;
                }
                break;
            }
        }

        droneX += std::cos(direction) * deltaPath;
        droneY += std::sin(direction) * deltaPath;

        // Перевірка влучання: дрон долетів до fire point
        if (state == MOVING && std::hypot(droneX - bestFireX, droneY - bestFireY) <= hitRadius * 0.25f)
        {
            break; // скид боєприпасу!
        }

        currentTime += simTimeStep;
        step++;
    }

    // --- Запис simulation.txt ---
    int N = step; // кількість кроків (індекс 0..N)

    std::ofstream fout(SIMULATION_PATH);
    if (!fout.is_open()) { std::cerr << "Cannot write simulation.txt" << std::endl; return 1; }

    // Рядок 1: N
    fout << N << std::endl;

    // Рядок 2: x0 y0 x1 y1 ... xN yN
    for (int i = 0; i <= N; i++) fout << outX[i] << " " << outY[i] << " "; fout << std::endl;
    for (int i = 0; i <= N; i++) fout << outDir[i] << " "; fout << std::endl;
    for (int i = 0; i <= N; i++) fout << outState[i] << " "; fout << std::endl;
    for (int i = 0; i <= N; i++) fout << outTarget[i] << " "; fout << std::endl;

    fout.close();

    std::ofstream fdout(DEBUG_PATH);
    if (!fdout.is_open()) { std::cerr << "Cannot write debug.txt" << std::endl; return 1; }

    for (int i = 0; i <= N; i++) fdout << outFireX[i] << " " << outFireY[i] << " "; fdout << std::endl;
    for (int i = 0; i <= N; i++) fdout << outPredTgtX[i] << " " << outPredTgtY[i] << " "; fdout << std::endl;

    fdout.close();

    return 0;
}
