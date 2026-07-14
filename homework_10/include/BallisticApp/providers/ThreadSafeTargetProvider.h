#pragma once

#include "BallisticApp/types/Coord.h"
#include "BallisticApp/providers/Target.h"
#include <filesystem>
#include <vector>
#include <mutex>
#include <atomic>
#include <chrono>

namespace BallisticApp {

class ThreadSafeTargetProvider {
public:
  using FloatSeconds = std::chrono::duration<float>;

  explicit ThreadSafeTargetProvider(std::filesystem::path filepath);
  ~ThreadSafeTargetProvider();

  ThreadSafeTargetProvider(const ThreadSafeTargetProvider&) = delete;
  ThreadSafeTargetProvider& operator=(const ThreadSafeTargetProvider&) = delete;

  bool load();
  void run();

  bool isThreadReady() const;
  void start();
  void stop();

  int getTargetCount() const;
  int getTimeSteps() const;

  // Отримання поточної позиції (real-time)
  Target getTarget(int targetIdx) const;
  std::vector<Target> getSnapshot() const;

  void setTiming(FloatSeconds arrayTimeStep, float timeScale);

  // Детерміноване отримання цілі у майбутньому часі відносно віртуального часу місії
  Target getTargetAtFutureTime(int targetIdx, float futureSeconds, float offsetSimTime) const;

private:
  void updateCurrentTargetsLocked(int stepIndex);
  Coord calcVelocityLocked(size_t targetIdx, size_t timeIdx) const;

  std::filesystem::path m_filepath;

  mutable std::mutex m_mutex;
  std::vector<std::vector<Coord>> m_paths;
  std::vector<Target> m_currentTargets;

  int m_targetCount{0};
  int m_timeSteps{0};

  FloatSeconds m_arrayTimeStep{0.1f};
  float m_timeScale{1.0f};

  std::atomic<bool> m_isReady{false};
  std::atomic<bool> m_isStarted{false};
  std::atomic<bool> m_stopRequested{false};

  static constexpr float MIN_TIME_SCALE = 1e-5f;
};

}  // namespace BallisticApp
