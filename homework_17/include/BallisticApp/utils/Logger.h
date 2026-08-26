#pragma once

#include <iostream>
#include <format>
#include <utility>
#include <string_view>
#include <string>

#define ENABLE_LOG 1
#define ENABLE_DEBUG 0

namespace BallisticApp {

template <typename... Args>
inline void log(std::format_string<Args...> format_str, Args&&... args) noexcept
{
#if ENABLE_LOG
  try {
    std::string message = std::format(format_str, std::forward<Args>(args)...);
    std::cout << std::format("[LOG] {:<65} [System]\n", message);
  }
  catch (...) {
    std::cout << "[LOG ERROR] Некоректний формат рядка логування.\n";
  }
#endif
}

template <typename... Args>
inline void log_with_mod(std::string_view module, std::format_string<Args...> format_str, Args&&... args) noexcept
{
#if ENABLE_LOG
  try {
    std::string message = std::format(format_str, std::forward<Args>(args)...);
    std::cout << std::format("[LOG] {:<65} [{}]\n", message, module);
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
    std::string message = std::format(format_str, std::forward<Args>(args)...);
    std::cout << std::format("[DEBUG] {:<65} [System]\n", message);
  }
  catch (...) {
  }
#endif
}

template <typename... Args>
inline void debug_with_mod([[maybe_unused]] std::string_view module,
                           [[maybe_unused]] std::format_string<Args...> format_str,
                           [[maybe_unused]] Args&&... args) noexcept
{
#if ENABLE_DEBUG
  try {
    std::string message = std::format(format_str, std::forward<Args>(args)...);
    std::cout << std::format("[DEBUG] {:<65} [{}]\n", message, module);
  }
  catch (...) {
  }
#endif
}

}  // namespace BallisticApp

#define APP_LOG_MOD(mod, fmt, ...) ::BallisticApp::log_with_mod(mod, fmt __VA_OPT__(, ) __VA_ARGS__)
#define APP_LOG(fmt, ...) ::BallisticApp::log(fmt __VA_OPT__(, ) __VA_ARGS__)

#define APP_DEBUG_MOD(mod, fmt, ...) ::BallisticApp::debug_with_mod(mod, fmt __VA_OPT__(, ) __VA_ARGS__)
#define APP_DEBUG(fmt, ...) ::BallisticApp::debug(fmt __VA_OPT__(, ) __VA_ARGS__)
