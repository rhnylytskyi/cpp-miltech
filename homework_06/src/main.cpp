#include "ballistics.hpp"

#include <iostream>

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "usage: ballistics_cli <input_path>\n";
        return 1;
    }

    try {
        BallisticsInput input = readInputData(argv[1]);
        DropSolution solution = computeDropSolution(input);

        if (solution.hasIntermediate) {
            std::cout << "Intermediate Point: (" << solution.intermediateX << ", " << solution.intermediateY << ")\n";
        }
        std::cout << "Fire Point: (" << solution.fireX << ", " << solution.fireY << ")\n";
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}
