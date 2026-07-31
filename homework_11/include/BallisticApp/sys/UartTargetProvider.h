#pragma once

#include "BallisticApp/utils/MathUtils.h"
#include "drone_link.h"
#include <vector>

namespace BallisticApp::sys {

/**
 * @brief Structure representing aggregated processed target tracking state.
 */
struct Target {
  Coord pos{0.0f, 0.0f};
  Coord velocity{0.0f, 0.0f};
};

/**
 * @brief Accumulates raw coordinate frames from UART and computes real-time target velocity tracking.
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

  /* Target trackers manage specific dynamic heaps and must not be copied */
  UartTargetProvider(const UartTargetProvider&) = delete;
  UartTargetProvider& operator=(const UartTargetProvider&) = delete;

  /**
   * @brief Pre-allocates tracking slots based on the active weapon capability configuration.
   */
  void setExpectedCount(uint8_t n);

  /**
   * @brief Updates internal state metrics and performs numerical differentiation to estimate target velocity vectors.
   */
  void update(const dlink::TargetPos& p, float nowSec);

  [[nodiscard]] bool hasTarget(int index) const noexcept;

  [[nodiscard]] int getTargetCount() const noexcept;

  [[nodiscard]] Target getTarget(int index) const noexcept;
};

}  // namespace BallisticApp::sys
