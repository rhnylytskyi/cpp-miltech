#include "BallisticApp/drivers/GpioPins.h"
#include <chrono>
#include <gpiod.h>
#include <stdexcept>
#include <thread>

namespace BallisticApp::sys {

void GpioPins::open(const std::string &chipName, unsigned int startLine, unsigned int dropLine)
{
  close();

  m_chip = gpiod_chip_open_by_name(chipName.c_str());

  if (!m_chip) {
    throw std::runtime_error("GpioPins: gpiod_chip_open_by_name('" + chipName + "') failed");
  }

  m_startLine = gpiod_chip_get_line(m_chip, startLine);
  m_dropLine = gpiod_chip_get_line(m_chip, dropLine);

  if (!m_startLine || !m_dropLine) {
    throw std::runtime_error("GpioPins: gpiod_chip_get_line failed");
  }

  if (gpiod_line_request_output(m_startLine, "drone", 0) != 0) {
    throw std::runtime_error("GpioPins: request_output(START) failed");
  }

  if (gpiod_line_request_output(m_dropLine, "drone", 0) != 0) {
    throw std::runtime_error("GpioPins: request_output(DROP) failed");
  }
}

void GpioPins::close()
{
  if (m_startLine) {
    gpiod_line_release(m_startLine);
    m_startLine = nullptr;
  }

  if (m_dropLine) {
    gpiod_line_release(m_dropLine);
    m_dropLine = nullptr;
  }

  if (m_chip) {
    gpiod_chip_close(m_chip);
    m_chip = nullptr;
  }
}

GpioPins::~GpioPins()
{
  close();
}

void GpioPins::setStart(bool value)
{
  if (!m_startLine) {
    throw std::runtime_error("GpioPins::setStart: not open");
  }

  gpiod_line_set_value(m_startLine, value ? 1 : 0);
}

void GpioPins::pulseDrop(int durationMs)
{
  if (!m_dropLine) {
    throw std::runtime_error("GpioPins::pulseDrop: not open");
  }

  gpiod_line_set_value(m_dropLine, 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(durationMs));
  gpiod_line_set_value(m_dropLine, 0);
}

}  // namespace BallisticApp::sys
