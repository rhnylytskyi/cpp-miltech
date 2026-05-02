#define _USE_MATH_DEFINES

#include <iostream>
#include <fstream>
#include <cmath>

int main(int argc, char** argv) {
    // Перевірка наявності аргументу командного рядка зі шляхом до файлу
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <input_path>\n";
        return 1;
    }

    // Відкриття вхідного файлу
    std::ifstream input_file(argv[1]);
    if (!input_file.is_open()) {
        std::cerr << "Error: Could not open file " << argv[1] << "\n";
        return 1;
    }

    // Параметри робота
    const int ticks_per_revolution = 1024;
    const double wheel_radius_m = 0.3;
    const double wheelbase_m = 1.0;

    // Початковий стан робота
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    // Змінні для зберігання даних попереднього кроку
    long prev_fl, prev_fr, prev_bl, prev_br;
    bool first_line = true;

    // Константа для переведення імпульсів у метри
    const double distance_per_tick = (2.0 * M_PI * wheel_radius_m) / ticks_per_revolution;

    long ts, fl, fr, bl, br;
    while (input_file >> ts >> fl >> fr >> bl >> br) {
        if (first_line) {
            // Зберігаємо початкові значення та пропускаємо перший крок (delta = 0)
            prev_fl = fl;
            prev_fr = fr;
            prev_bl = bl;
            prev_br = br;
            first_line = false;
            continue;
        }

        // Крок 1: Delta імпульсів по кожному колесу
        long d_fl = fl - prev_fl;
        long d_fr = fr - prev_fr;
        long d_bl = bl - prev_bl;
        long d_br = br - prev_br;

        // Крок 2: Усереднення бортів
        double d_left = (static_cast<double>(d_fl) + d_bl) / 2.0;
        double d_right = (static_cast<double>(d_fr) + d_br) / 2.0;

        // Крок 3: Переведення імпульсів у метри
        double dL = d_left * distance_per_tick;
        double dR = d_right * distance_per_tick;

        // Крок 4: Розрахунок відстані центру та зміни орієнтації
        double d = (dL + dR) / 2.0;
        double dtheta = (dR - dL) / wheelbase_m;

        // Крок 5: Оновлення позиції (midpoint integration)
        x += d * std::cos(theta + dtheta / 2.0);
        y += d * std::sin(theta + dtheta / 2.0);
        theta += dtheta;

        // Вивід траєкторії на stdout
        std::cout << ts << " " << x << " " << y << " " << theta << "\n";

        // Оновлення значень для наступної ітерації
        prev_fl = fl;
        prev_fr = fr;
        prev_bl = bl;
        prev_br = br;
    }

    input_file.close();
    return 0;
}
