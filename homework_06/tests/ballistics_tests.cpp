#include "ballistics.hpp"

#include <gtest/gtest.h>
#include <stdexcept>
#include <fstream>
#include <cstdio>
#include <cmath>

// Перевірка розрахунку балістики для відомого випадку
TEST(Ballistics, ComputesKnownDropPoint) {
  const BallisticsInput input{
      .droneX = 100.0,
      .droneY = 100.0,
      .droneZ = 100.0,
      .targetX = 200.0,
      .targetY = 200.0,
      .attackSpeed = 10.0,
      .accelerationPath = 10.0,
      .ammoName = "VOG-17",
  };

  const DropSolution solution = computeDropSolution(input);

  EXPECT_NEAR(solution.fireX, 175.441, 0.01);
  EXPECT_NEAR(solution.fireY, 175.441, 0.01);
}

// Перевірка розрахунку балістики для випадку, коли потрібна проміжна точка
TEST(Ballistics, ComputesIntermediatePoint) {
  const BallisticsInput input{
      .droneX = 145.0,
      .droneY = 145.0,
      .droneZ = 100.0,
      .targetX = 150.0,
      .targetY = 150.0,
      .attackSpeed = 10.0,
      .accelerationPath = 10.0,
      .ammoName = "VOG-17",
  };

  const DropSolution solution = computeDropSolution(input);

  EXPECT_TRUE(solution.hasIntermediate);
  EXPECT_NEAR(solution.intermediateX, 118.370, 0.01);
  EXPECT_NEAR(solution.intermediateY, 118.370, 0.01);
  EXPECT_NEAR(solution.fireX, 125.441, 0.01);
  EXPECT_NEAR(solution.fireY, 125.441, 0.01);
}

// Перевірка розрахунку балістики для випадку, коли дрон над ціллю
TEST(Ballistics, ComputesWhenDroneAboveTarget) {
  const BallisticsInput input{
      .droneX = 150.0,
      .droneY = 150.0,
      .droneZ = 100.0,
      .targetX = 150.0,
      .targetY = 150.0,
      .attackSpeed = 10.0,
      .accelerationPath = 10.0,
      .ammoName = "VOG-17",
  };

  const DropSolution solution = computeDropSolution(input);

  EXPECT_TRUE(solution.hasIntermediate);
  EXPECT_NEAR(solution.intermediateX, 105.268, 0.01);
  EXPECT_NEAR(solution.intermediateY, 150.0, 0.01);
  EXPECT_NEAR(solution.fireX, 115.268, 0.01);
  EXPECT_NEAR(solution.fireY, 150.0, 0.01);
}

// Перевірка повідомлення, якщо файл не існує
TEST(Ballistics, VerifiesNonExistentFileMessage) {
    try {
        readInputData("this_file_definitely_does_not_exist.txt");
        FAIL() << "Expected std::runtime_error but no exception was thrown.";
    } 
    catch (const std::runtime_error& err) {
        EXPECT_STREQ(err.what(), "Cannot open input file.");
    } 
    catch (...) {
        FAIL() << "Expected std::runtime_error but caught a different exception type.";
    }
}

// Перевірка повідомлення при пошкодженому форматі даних
TEST(Ballistics, VerifiesInvalidFormatMessage) {
    const std::string tempFilename = "temp_corrupted_test_input.txt";
    
    std::ofstream fout(tempFilename);
    fout << "TEXT_INSTEAD_OF_NUMBER 100.0 100.0 200.0 200.0 10.0 10.0 VOG-17\n";
    fout.close();

    try {
        readInputData(tempFilename);
        std::remove(tempFilename.c_str()); // Очищення, якщо тест раптом не впав
        FAIL() << "Expected std::runtime_error due to invalid format.";
    } 
    catch (const std::runtime_error& err) {
        std::remove(tempFilename.c_str()); // Обов'язково видаляємо файл
        // Перевіряємо точний збіг повідомлення про формат
        EXPECT_STREQ(err.what(), "Invalid data format in input file.");
    } 
    catch (...) {
        std::remove(tempFilename.c_str());
        FAIL() << "Expected std::runtime_error but caught something else.";
    }
}

// Перевірка повідомлення для невідомої бомби (функція computeDropSolution)
TEST(Ballistics, VerifiesUnknownAmmoMessage) {
    BallisticsInput input{
        .droneX = 100.0, .droneY = 100.0, .droneZ = 100.0,
        .targetX = 200.0, .targetY = 200.0,
        .attackSpeed = 10.0, .accelerationPath = 10.0,
        .ammoName = "UNKNOWN_BOMB"
    };

    try {
        computeDropSolution(input);
        FAIL() << "Expected std::runtime_error for unknown ammo.";
    } 
    catch (const std::runtime_error& err) {
        // Перевіряємо динамічно згенерований рядок
        EXPECT_STREQ(err.what(), "Unknown ammo name: UNKNOWN_BOMB");
    }
}

// Перевірка успішного зчитування коректного файлу
TEST(Ballistics, ReadsValidFileCorrectly) {
    const std::string tempFilename = "temp_valid_test_input.txt";
    
    // Створюємо тимчасовий файл із коректними даними
    std::ofstream fout(tempFilename);
    fout << "100.0 100.0 100.0 200.0 200.0 10.0 10.0 VOG-17\n";
    fout.close();

    const BallisticsInput input = readInputData(tempFilename);

    EXPECT_FLOAT_EQ(input.droneX, 100.0f);
    EXPECT_FLOAT_EQ(input.droneY, 100.0f);
    EXPECT_FLOAT_EQ(input.droneZ, 100.0f);
    EXPECT_FLOAT_EQ(input.targetX, 200.0f);
    EXPECT_FLOAT_EQ(input.targetY, 200.0f);
    EXPECT_FLOAT_EQ(input.attackSpeed, 10.0f);
    EXPECT_FLOAT_EQ(input.accelerationPath, 10.0f);
    EXPECT_EQ(input.ammoName, "VOG-17");

    std::remove(tempFilename.c_str());
}

// Перевірка, що планеруючий боєприпас дає скінченний додатний час падіння
TEST(Ballistics, ReturnsFinitePositiveTimeForGlidingAmmo) {
    const float z0   = 100.0f;
    const float v0   = 10.0f;
    const float mass = 0.45f;
    const float drag = 0.10f;
    const float lift = 1.0f; // Планеруючий боєприпас

    float flightTime = calcTimeOfFall(z0, v0, mass, drag, lift);

    EXPECT_GT(flightTime, 0.0f);           // Час має бути строго більшим за нуль
    EXPECT_FALSE(std::isnan(flightTime));  // Результат не повинен бути NaN
    EXPECT_FALSE(std::isinf(flightTime));  // Результат не повинен бути нескінченністю
}

// Перевірка на виключення при некоректній висоті
TEST(Ballistics, ThrowsExceptionWhenHeightIsZeroOrNegative) {
    const float v0   = 10.0f;
    const float mass = 0.45f;
    const float drag = 0.10f;
    const float lift = 1.0f; // Планеруючий боєприпас

    EXPECT_THROW(calcTimeOfFall(0.0f, v0, mass, drag, lift), std::runtime_error);
    EXPECT_THROW(calcTimeOfFall(-10.0f, v0, mass, drag, lift), std::runtime_error);
}
