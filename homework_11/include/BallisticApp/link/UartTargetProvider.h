#pragma once
#include "BallisticApp/types/Coord.h"
#include "third_party/drone_link.h"
#include <vector>

namespace BallisticApp {

struct Target {
  Coord pos{0.0f, 0.0f};
  Coord velocity{0.0f, 0.0f};
};

class UartTargetProvider {
private:
  struct Entry {
    Coord pos{0.0f, 0.0f};
    Coord vel{0.0f, 0.0f};
    float lastT{0.0f};
    bool known{false};
  };

  std::vector<Entry> entries_;

public:
  UartTargetProvider() = default;
  ~UartTargetProvider() = default;

  UartTargetProvider(const UartTargetProvider&) = delete;
  UartTargetProvider& operator=(const UartTargetProvider&) = delete;

  void setExpectedCount(uint8_t n);
  void update(const dlink::TargetPos& p, float nowSec);
  bool hasTarget(int index) const;
  int getTargetCount();

  Target getTarget(int index);
};

}  // namespace BallisticApp
