#include "BallisticApp/hardware/I2cDevice.h"
#include "BallisticApp/hardware/Mpu6050.h"
#include "BallisticApp/utils/Logger.h"

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <span>
#include <algorithm>
#include <stdexcept>

using namespace BallisticApp;

namespace {

struct AppArgs {
  std::string busPath = "/dev/i2c-1";
  int address = Mpu6050::kDefaultAddress;
  int count = -1;
  int rateHz = 5;
};

void printHelp()
{
  std::cout << "===================================================================\n"
            << "  BallisticApp MPU-6050 I2C Sensor Interface CLI\n"
            << "===================================================================\n"
            << "Usage:\n"
            << "  ./homework_16 [bus_path] [address] [options]\n\n"
            << "Options:\n"
            << "  --count <N>   Number of samples to read (Default: infinite)\n"
            << "  --rate <N>    Sampling rate frequency in Hz (Default: 5)\n"
            << "  -h, --help    Display documentation metrics\n"
            << "===================================================================\n";
}

void parseArgs(std::span<const char* const> args, AppArgs& out)
{
  auto helpIt = std::find_if(args.begin(), args.end(), [](std::string_view arg) { return arg == "--help" || arg == "-h"; });

  if (helpIt != args.end()) {
    printHelp();
    std::exit(0);
  }

  // Parse count override parameter
  auto countIt = std::find_if(args.begin(), args.end(), [](std::string_view arg) { return arg == "--count"; });
  if (countIt != args.end() && std::next(countIt) != args.end()) {
    out.count = std::atoi(*std::next(countIt));
  }

  // Parse rate override frequency parameter
  auto rateIt = std::find_if(args.begin(), args.end(), [](std::string_view arg) { return arg == "--rate"; });
  if (rateIt != args.end() && std::next(rateIt) != args.end()) {
    out.rateHz = std::atoi(*std::next(rateIt));
  }

  // Positional parameters extraction
  int positionalIndex = 0;
  for (size_t i = 1; i < args.size(); ++i) {
    std::string_view currentArg = args[i];
    if (currentArg.starts_with("-")) {
      if (currentArg == "--count" || currentArg == "--rate") {
        ++i;  // Skip associated arguments value block
      }
      continue;
    }

    if (positionalIndex == 0) {
      out.busPath = currentArg;
      ++positionalIndex;
    }
    else if (positionalIndex == 1) {
      out.address = static_cast<int>(std::strtol(currentArg.data(), nullptr, 0));
      ++positionalIndex;
    }
  }

  if (out.rateHz < 1)
    out.rateHz = 1;
}

}  // namespace

int main(int argc, char* argv[])
{
  try {
    std::span<const char* const> spanArgs(argv, argc);
    AppArgs appArgs;
    parseArgs(spanArgs, appArgs);

    APP_LOG_MOD("Main", "Initializing hardware interface bus context...");
    APP_LOG("{:.<15} {}", "I2C_BUS", appArgs.busPath);
    APP_LOG("{:.<15} 0x{:02X}", "DEVICE_ADDR", appArgs.address);
    APP_LOG("{:.<15} {} Hz", "SAMPLING_RATE", appArgs.rateHz);

    I2cDevice device(appArgs.busPath, appArgs.address);
    Mpu6050 sensor(device);

    sensor.checkWhoAmI();
    sensor.wake();

    APP_LOG_MOD("Main", "SUCCESS | MPU-6050 hardware validation verified. Executing sensor stream.");

    const auto interval = std::chrono::milliseconds(1000 / appArgs.rateHz);

    for (int step = 0; appArgs.count < 0 || step < appArgs.count; ++step) {
      const auto sample = sensor.readSample();

      APP_LOG_MOD("Sensor",
                  "Accel: [{: .3f}, {: .3f}, {: .3f}] g | Gyro: [{: 7.2f}, {: 7.2f}, {: 7.2f}] dps | Temp: {:.1f} C",
                  sample.accelX_g,
                  sample.accelY_g,
                  sample.accelZ_g,
                  sample.gyroX_dps,
                  sample.gyroY_dps,
                  sample.gyroZ_dps,
                  sample.temperatureC);

      std::this_thread::sleep_for(interval);
    }
  }
  catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
