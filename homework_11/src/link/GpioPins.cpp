#include "BallisticApp/link/GpioPins.h"
#include <chrono>
#include <gpiod.h>
#include <stdexcept>
#include <thread>

void GpioPins::open(const std::string &chipName, unsigned int startLine, unsigned int dropLine)
{
  close();

  chip_ = gpiod_chip_open_by_name(chipName.c_str());

  if (!chip_) {
    throw std::runtime_error("GpioPins: gpiod_chip_open_by_name('" + chipName + "') failed");
  }

  startLine_ = gpiod_chip_get_line(chip_, startLine);
  dropLine_ = gpiod_chip_get_line(chip_, dropLine);

  if (!startLine_ || !dropLine_) {
    throw std::runtime_error("GpioPins: gpiod_chip_get_line failed");
  }

  // Обидві лінії — виходи, стартове значення 0 (не готові / без імпульсу).
  if (gpiod_line_request_output(startLine_, "drone", 0) != 0) {
    throw std::runtime_error("GpioPins: request_output(START) failed");
  }

  if (gpiod_line_request_output(dropLine_, "drone", 0) != 0) {
    throw std::runtime_error("GpioPins: request_output(DROP) failed");
  }
}

void GpioPins::close()
{
  if (startLine_) {
    gpiod_line_release(startLine_);
    startLine_ = nullptr;
  }

  if (dropLine_) {
    gpiod_line_release(dropLine_);
    dropLine_ = nullptr;
  }

  if (chip_) {
    gpiod_chip_close(chip_);
    chip_ = nullptr;
  }
}

GpioPins::~GpioPins()
{
  close();
}

void GpioPins::setStart(bool value)
{
  if (!startLine_) {
    throw std::runtime_error("GpioPins::setStart: not open");
  }

  gpiod_line_set_value(startLine_, value ? 1 : 0);
}

void GpioPins::pulseDrop(int durationMs)
{
  if (!dropLine_) {
    throw std::runtime_error("GpioPins::pulseDrop: not open");
  }

  gpiod_line_set_value(dropLine_, 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(durationMs));
  gpiod_line_set_value(dropLine_, 0);
}
