#pragma once

#include "BallisticApp/utils/MathUtils.h"
#include "drone_link.h"
#include <vector>

namespace BallisticApp {

struct Target {
  Coord pos{0.0f, 0.0f};
  Coord velocity{0.0f, 0.0f};
};

}  // namespace BallisticApp

namespace BallisticApp {

class UartTargetProvider {
public:
  UartTargetProvider() = default;
  ~UartTargetProvider() = default;

  UartTargetProvider(const UartTargetProvider&) = delete;
  UartTargetProvider& operator=(const UartTargetProvider&) = delete;

  void setExpectedCount(uint8_t n);
  void update(const dlink::TargetPos& p, float nowSec);

  [[nodiscard]] bool hasTarget(int index) const noexcept;
  [[nodiscard]] int getTargetCount() const noexcept;
  [[nodiscard]] Target getTarget(int index) const noexcept;

private:
  struct Entry {
    Coord pos{0.0f, 0.0f};
    Coord vel{0.0f, 0.0f};
    float lastT{0.0f};
    bool known{false};
  };

  std::vector<Entry> m_entries;
};

}  // namespace BallisticApp
