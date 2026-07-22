#include "BallisticApp/link/UartTargetProvider.h"

namespace BallisticApp {

void UartTargetProvider::setExpectedCount(uint8_t n)
{
  if (entries_.size() < n) {
    entries_.resize(n);
  }
}

void UartTargetProvider::update(const dlink::TargetPos &p, float nowSec)
{
  if (entries_.size() <= p.id) {
    entries_.resize((size_t)p.id + 1);
  }

  Entry &e = entries_[p.id];
  Coord newPos{p.x, p.y};

  if (e.known) {
    float dt = nowSec - e.lastT;
    if (dt > 1e-3f) {
      e.vel = (newPos - e.pos) / dt;
    }
  }
  else {
    e.vel = {0.0f, 0.0f};
  }

  e.pos = newPos;
  e.lastT = nowSec;
  e.known = true;
}

bool UartTargetProvider::hasTarget(int index) const
{
  return index >= 0 && (size_t)index < entries_.size() && entries_[index].known;
}

int UartTargetProvider::getTargetCount()
{
  return (int)entries_.size();
}

Target UartTargetProvider::getTarget(int index)
{
  if (index < 0 || (size_t)index >= entries_.size()) {
    return Target{{0.0f, 0.0f}, {0.0f, 0.0f}};
  }

  const Entry &e = entries_[index];
  return Target{e.pos, e.vel};
}

}  // namespace BallisticApp
