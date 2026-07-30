#include "BallisticApp/mission/Autopilot.h"
#include "BallisticApp/drivers/GpioPins.h"
#include "BallisticApp/drivers/UartLink.h"
#include "BallisticApp/utils/AppArguments.h"
#include "BallisticApp/ComponentFactory.h"
#include "BallisticApp/utils/Logger.h"
#include "drone_link.h"
#include <span>
#include <iostream>
#include <memory>
#include <thread>

using namespace BallisticApp;

int main(int argc, char* argv[])
{
  using UartLink = BallisticApp::sys::UartLink;
  using GpioPins = BallisticApp::sys::GpioPins;
  using Autopilot = BallisticApp::mission::Autopilot;

  try {
    std::span<const char* const> spanArgs(argv, argc);
    AppArguments appArgs(spanArgs);

    APP_LOG("{:.<15}{}", "UART_PORT", appArgs.getUart());
    APP_LOG("{:.<15}{}", "GPIO_CHIP", appArgs.getGpioChip());
    APP_LOG("{:.<15}START={},DROP={}", "GPIO_LINES", appArgs.getStartLine(), appArgs.getDropLine());

    UartLink link;
    link.open(appArgs.getUart());

    GpioPins gpio;
    gpio.open(appArgs.getGpioChip(), appArgs.getStartLine(), appArgs.getDropLine());

    // === КРИТИЧНИЙ ФІКС №1: Вигрібаємо старе сміття з буфера UART ===
    std::cerr << "[SYSTEM] Очищення буферів UART від бруду попередніх запусків...\n";
    while (link.pump() > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    link.resetParser();  // Повністю скидаємо стан скінченного автомата

    dlink::AmmoCfg ammo{};
    dlink::DroneCfg cfg{};
    bool haveAmmo = false;
    bool haveCfg = false;

    // === КРИТИЧНИЙ ФІКС №2: Налаштовуємо «вуха» ДО того, як смикати START ===
    link.onAmmo([&](const dlink::AmmoCfg& a) {
      ammo = a;
      haveAmmo = true;
      std::cerr << "[MAIN DETECT] Успішно зловлено пакет AMMO: " << a.name << "\n";
    });

    link.onConfig([&](const dlink::DroneCfg& c) {
      cfg = c;
      haveCfg = true;
      std::cerr << "[MAIN DETECT] Успішно зловлено пакет CONFIG! attackSpeed=" << c.attackSpeed << "\n";
    });

    // === КРИТИЧНИЙ ФІКС №3: Тільки тепер піднімаємо лінію START ===
    std::cerr << "[SYSTEM] Даємо чекеру сигнал START...\n";
    gpio.setStart(true);

    // Локальний цикл очікування handshake безпосередньо у main
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(5000);
    while ((!haveAmmo || !haveCfg) && std::chrono::steady_clock::now() < deadline) {
      link.pump();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cerr << "[DEBUG ARCH] Розмір DroneCfg у пам'яті: " << sizeof(dlink::DroneCfg) << " байт\n";
    if (!haveAmmo || !haveCfg) {
      std::cerr << "student: handshake failed — ammo=" << haveAmmo << " cfg=" << haveCfg << "\n";
      return 1;
    }

    APP_LOG_MOD("Main", "SUCCESS | Handshake finalized. Loading ballistic solver...");

    auto solver = ComponentFactory::createSolver(SolverType::TABLE);
    std::string tablePath = std::string(PROJECT_DATA_DIR) + "/ballistic_table.txt";
    std::cerr << "[MAIN] Завантаження балістичної таблиці з: " << tablePath << "\n";
    if (!solver->initialize(tablePath)) {
      throw std::runtime_error("Critical: Failed to initialize Ballistic Table Solver.");
    }

    Autopilot autopilot(link, gpio, std::move(solver), ammo, cfg);

    APP_LOG_MOD("Main", "student: flying...");

    while (autopilot.step()) {
    }

    APP_LOG_MOD("Main", "student: done (dropped={})", autopilot.dropped() ? "yes" : "no");
  }
  catch (const std::exception& e) {
    std::cerr << "student: error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
