#include "ballistics.hpp"

#include <iostream>
#include <fstream>
#include <cstring>
#include <iomanip>
#include <filesystem>
#include <string>

namespace fs = std::filesystem;

// Структура для збереження вхідних даних
struct InputData {
    float xd, yd, zd;
    float targetX, targetY;
    float attackSpeed;
    float accelerationPath;
    std::string ammoName;
};

// Структура для збереження результатів розрахунку
struct BallisticsResult {
    float intermediateX = 0.0f;
    float intermediateY = 0.0f;
    float fireX = 0.0f;
    float fireY = 0.0f;
    bool hasIntermediate = false;
};

// 1. ФУНКЦІЯ ЧИТАННЯ ВХІДНИХ ДАНИХ
bool readInputData(const std::string& filename, InputData& data) {
    std::ifstream fin(filename);
    if (!fin.is_open()) {
        std::cerr << "Error: Cannot open input file: " << filename << "\n";
        return false;
    }

    if (!(fin >> data.xd >> data.yd >> data.zd 
            >> data.targetX >> data.targetY 
            >> data.attackSpeed 
            >> data.accelerationPath 
            >> data.ammoName)) {
        std::cerr << "Error: Invalid data format in input file.\n";
        return false;
    }

    fin.close();
    return true;
}

// 2. ФУНКЦІЯ ОБЧИСЛЕННЯ БАЛІСТИКИ
bool calculateBallistics(const InputData& input, BallisticsResult& result) {
    // Шукаємо індекс боєприпасу за назвою
    int bombIdx = findBombIndexByName(input.ammoName);
    if (bombIdx < 0) {
        std::cerr << "Error: Unknown ammo name: " << input.ammoName << "\n";
        return false;
    }

    // Отримуємо фізичні константи боєприпасу з ваших масивів у hpp
    float m = bombM[bombIdx];
    float d = bombD[bombIdx];
    float l = bombL[bombIdx];

    // Розрахунок балістичної траєкторії (ваші бібліотечні функції)
    float flightTime = calcTimeOfFall(input.zd, input.attackSpeed, m, d, l);
    float hDist = calcHDistance(flightTime, input.attackSpeed, m, d, l);

    // Отримання фінальних точок
    getIntermediateAndDropPoint(
        input.xd, input.yd, input.targetX, input.targetY, hDist, input.accelerationPath,
        result.intermediateX, result.intermediateY, result.fireX, result.fireY, result.hasIntermediate
    );

    return true;
}

// 3. ФУНКЦІЯ ВИЗНАЧЕННЯ ШЛЯХУ ТА ЗАПИСУ РЕЗУЛЬТАТУ
bool writeOutputData(const fs::path& inputPath, const BallisticsResult& result) {
    fs::path baseDir;

    // Визначаємо кореневу папку на основі шляху до вхідного файлу
    if (inputPath.has_parent_path() && inputPath.parent_path().has_parent_path()) {
        baseDir = inputPath.parent_path().parent_path();
    } else {
        fs::path currentDir = fs::current_path();
        baseDir = fs::exists(currentDir / "homework_06") ? currentDir / "homework_06" : currentDir;
    }

    fs::path outputDir = baseDir / "out";
    fs::create_directories(outputDir); // Автоматично створюємо 'out', якщо її немає
    fs::path outputPath = outputDir / "output.txt";

    std::ofstream outputFile(outputPath);
    if (!outputFile.is_open()) {
        std::cerr << "Error: Cannot open or create output file at " << outputPath << "\n";
        return false;
    }

    // Форматований запис результатів
    outputFile << std::fixed << std::setprecision(3);
    if (result.hasIntermediate) {
        outputFile << result.intermediateX << " " << result.intermediateY << " ";
    }
    outputFile << result.fireX << " " << result.fireY << "\n";
    
    outputFile.close();
    return true;
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "usage: ballistics_cli <input_path>\n";
        return 1;
    }

    InputData input;
    BallisticsResult result;

    // Крок 1: Читання
    if (!readInputData(argv[1], input)) {
        return 1;
    }

    // Крок 2: Обчислення
    if (!calculateBallistics(input, result)) {
        return 1;
    }

    // Крок 3: Запис
    if (!writeOutputData(argv[1], result)) {
        return 1;
    }

    std::cout << "Success: Ballistics calculated and saved to out/output.txt\n";
    return 0;
}
