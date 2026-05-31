#pragma once

#include <iostream>
#include <string_view>
#include <format>
#include <utility>

#define ENABLE_LOG 1
#define ENABLE_DEBUG 0

namespace BallisticApp::Log {

template <typename... Args>
inline void log(std::format_string<Args...> format_str, Args&&... args) noexcept
{
#if ENABLE_LOG
  try {
    std::cout << "[LOG] " << std::format(format_str, std::forward<Args>(args)...) << '\n';
  }
  catch (...) {
    std::cout << "[LOG ERROR] Некоректний формат рядка логування.\n";
  }
#endif
}

template <typename... Args>
inline void debug([[maybe_unused]] std::format_string<Args...> format_str, [[maybe_unused]] Args&&... args) noexcept
{
#if ENABLE_DEBUG
  try {
    std::cout << "[DEBUG] " << std::format(format_str, std::forward<Args>(args)...) << '\n';
  }
  catch (...) {
  }
#endif
}

}  // namespace BallisticApp::Log

#define APP_LOG(fmt, ...) ::BallisticApp::Log::log(fmt __VA_OPT__(,) __VA_ARGS__)
#define APP_DEBUG(fmt, ...) ::BallisticApp::Log::debug(fmt __VA_OPT__(,) __VA_ARGS__)
