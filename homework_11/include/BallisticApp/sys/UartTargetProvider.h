#pragma once

#include "BallisticApp/utils/MathUtils.h"
#include "drone_link.h"
#include <vector>

namespace BallisticApp::sys {

struct Target {
  Coord pos{0.0f, 0.0f};
  Coord velocity{0.0f, 0.0f};
};

/**
 * @brief Accumulates UART coordinates and estimates target velocity vectors.
 */
class UartTargetProvider {
private:
  struct Entry {
    Coord pos{0.0f, 0.0f};
    Coord vel{0.0f, 0.0f};
    float lastT{0.0f};
    bool known{false};
  };

  std::vector<Entry> m_entries;

public:
  UartTargetProvider() = default;
  ~UartTargetProvider() = default;

  /* Prevent tracking storage duplication */
  UartTargetProvider(const UartTargetProvider&) = delete;
  UartTargetProvider& operator=(const UartTargetProvider&) = delete;

  /* State tracking and estimation math */
  void setExpectedCount(uint8_t n);
  void update(const dlink::TargetPos& p, float nowSec);

  /* Inspector metrics */
  [[nodiscard]] bool hasTarget(int index) const noexcept;
  [[nodiscard]] int getTargetCount() const noexcept;
  [[nodiscard]] Target getTarget(int index) const noexcept;
};

}  // namespace BallisticApp::sys
