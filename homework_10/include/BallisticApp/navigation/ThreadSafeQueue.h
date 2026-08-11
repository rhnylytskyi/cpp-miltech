#pragma once
#include <mutex>
#include <queue>
#include <utility>
#include <optional>

namespace BallisticApp {

template <typename T>
class ThreadSafeQueue {
public:
  ThreadSafeQueue() = default;
  ~ThreadSafeQueue() = default;

  void push(T value)
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_queue.push(std::move(value));
  }

  std::optional<T> tryPop()
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_queue.empty()) {
      return std::nullopt;
    }
    T value = std::move(m_queue.front());
    m_queue.pop();
    return value;
  }

  bool empty() const
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_queue.empty();
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::queue<T> emptyQueue;
    std::swap(m_queue, emptyQueue);
  }

private:
  std::queue<T> m_queue;
  mutable std::mutex m_mutex;
};

}  // namespace BallisticApp
