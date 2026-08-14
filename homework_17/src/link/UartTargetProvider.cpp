#include "BallisticApp/link/UartTargetProvider.h"
#include <cmath>
#include <mutex>

namespace BallisticApp::link {

/* Shared synchronization primitives for thread-safe target container operations */
static std::mutex s_targetsMutex;

void UartTargetProvider::setExpectedCount(uint8_t n)
{
  std::lock_guard<std::mutex> lock(s_targetsMutex);
  if (m_entries.size() < n) {
    m_entries.resize(n);
  }
}

void UartTargetProvider::update(const dlink::TargetPos& p, float nowSec)
{
  std::lock_guard<std::mutex> lock(s_targetsMutex);
  if (m_entries.size() <= p.id) {
    m_entries.resize(static_cast<size_t>(p.id) + 1);
  }

  Entry& e = m_entries[p.id];
  Coord newPos{p.x, p.y};

  if (e.known) {
    float dt = nowSec - e.lastT;
    constexpr float kMinDeltaTimeSec = 1e-3f; /* Protect against division by near-zero step clocks */
    if (dt > kMinDeltaTimeSec) {
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

bool UartTargetProvider::hasTarget(int index) const noexcept
{
  std::lock_guard<std::mutex> lock(s_targetsMutex);
  return index >= 0 && static_cast<size_t>(index) < m_entries.size() && m_entries[index].known;
}

int UartTargetProvider::getTargetCount() const noexcept
{
  std::lock_guard<std::mutex> lock(s_targetsMutex);
  return static_cast<int>(m_entries.size());
}

Target UartTargetProvider::getTarget(int index) const noexcept
{
  std::lock_guard<std::mutex> lock(s_targetsMutex);
  if (index < 0 || static_cast<size_t>(index) >= m_entries.size()) {
    return Target{{0.0f, 0.0f}, {0.0f, 0.0f}};
  }

  const Entry& e = m_entries[index];
  return Target{e.pos, e.vel};
}

}  // namespace BallisticApp::link
