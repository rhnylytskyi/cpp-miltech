#define _USE_MATH_DEFINES
#include <iostream>
#include <fstream>
#include <cmath>
#include <cstring>
#include <iomanip>

int main_() {
    float xd, yd, zd;
    float targetX, targetY;
    float attackSpeed, accelerationPath;
    char ammo_name[20];
    float m; // маса (кг)
    float d; // drag (опір)
    float l; // lift (підйомна сила)

    std::ifstream inputFile("input.txt");
    if (!inputFile.is_open()) {
        std::cout << "Error: Cannot open input.txt" << std::endl;
        return 1;
    }

    inputFile >> xd >> yd >> zd >> targetX >> targetY >> attackSpeed >> accelerationPath >> ammo_name;
    inputFile.close();

    bool found = false;
    if (strcmp(ammo_name, "VOG-17") == 0) { m = 0.35f; d = 0.07f; l = 0.0f; found = true; }
    else if (strcmp(ammo_name, "M67") == 0) { m = 0.6f; d = 0.10f; l = 0.0f; found = true; }
    else if (strcmp(ammo_name, "RKG-3") == 0) { m = 1.2f; d = 0.10f; l = 0.0f; found = true; }
    else if (strcmp(ammo_name, "GLIDING-VOG") == 0) { m = 0.45f; d = 0.10f; l = 1.0f; found = true; }
    else if (strcmp(ammo_name, "GLIDING-RKG") == 0) { m = 1.4f; d = 0.10f; l = 1.0f; found = true; }

    if (!found) {
        std::cout << "Error: Unknown ammo type" << std::endl;
        return 1;
    }

    const double g = 9.81;

    double a_coeff = d * g * m - 2 * pow(d, 2) * l * attackSpeed;
    double b_coeff = -3 * g * pow(m, 2) + 3 * d * l * m * attackSpeed;
    double c_coeff = 6 * pow(m, 2) * zd;
	std::cout << "Coefficients: a=" << a_coeff << ", b=" << b_coeff << ", c=" << c_coeff << std::endl;

    double p = -pow(b_coeff, 2) / (3 * pow(a_coeff, 2));
	double q = (2 * pow(b_coeff, 3)) / (27 * pow(a_coeff, 3)) + c_coeff / a_coeff;
	std::cout << "Coefficients: p=" << p << ", q=" << q << std::endl;

    double phi_arg = (3 * q / (2 * p)) * sqrt(-3 / p);

    // Перевірка діапазону арккосинуса
    if (phi_arg < -1.0) phi_arg = -1.0;
    if (phi_arg > 1.0) phi_arg = 1.0;

    double phi = acos(phi_arg);
	std::cout << "Calculated phi: " << phi << std::endl;
    double t = 2 * sqrt(-p / 3) * cos((phi + 4 * M_PI) / 3) - b_coeff / (3 * a_coeff);
	std::cout << "Calculated time of flight (t): " << t << std::endl;

    double v0 = attackSpeed;
    double h = v0 * t
        - (pow(t, 2) * d * v0) / (2 * m)
        + (pow(t, 3) * (6 * d * g * l * m - 6 * pow(d, 2) * (pow(l, 2) - 1) * v0)) / (36 * pow(m, 2))
        + (pow(t, 4) * (-6 * pow(d, 2) * g * l * (1 + pow(l, 2) + pow(l, 4)) * m + 3 * pow(d, 3) * pow(l, 2) * (1 + pow(l, 2)) * v0 + 6 * pow(d, 3) * pow(l, 4) * (1 + pow(l, 2)) * v0)) / (36 * pow(1 + pow(l, 2), 2) * pow(m, 3))
        + (pow(t, 5) * (3 * pow(d, 3) * g * pow(l, 3) * m - 3 * pow(d, 4) * pow(l, 2) * (1 + pow(l, 2)) * v0)) / (36 * (1 + pow(l, 2)) * pow(m, 4));
    std::cout << "Calculated horizontal distance (h): " << h << std::endl;

    double D = sqrt(pow(targetX - xd, 2) + pow(targetY - yd, 2));
	std::cout << "Distance to target (D): " << D << std::endl;

    std::ofstream outputFile("output.txt");
    outputFile << std::fixed << std::setprecision(3);

    if (h + accelerationPath > D) {
		std::cout << "Warning: Acceleration path exceeds distance to target. Adjusting aim point." << std::endl;
        double xd_prime = targetX - (targetX - xd) * (h + accelerationPath) / D;
        double yd_prime = targetY - (targetY - yd) * (h + accelerationPath) / D;
        outputFile << xd_prime << " " << yd_prime << " ";
		std::cout << "Adjusted aim point: (" << xd_prime << ", " << yd_prime << ")" << std::endl;
    }

    double ratio = (D - h) / D;
    double fireX = xd + (targetX - xd) * ratio;
    double fireY = yd + (targetY - yd) * ratio;

    outputFile << fireX << " " << fireY << std::endl;
	std::cout << "Calculated fire point: (" << fireX << ", " << fireY << ")" << std::endl;
    outputFile.close();

    return 0;
}
