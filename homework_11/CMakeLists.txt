cmake_minimum_required(VERSION 3.20)
project(BallisticAutopilot)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Threads REQUIRED)

# Шукаємо саме C-бібліотеку gpiod
find_library(GPIOD_LIBRARY NAMES gpiod)

# Якщо ми локально в контейнері — лінкуємо її, якщо на GitHub (де немає інтернету) — створюємо віртуальну лінку
if(NOT GPIOD_LIBRARY)
    message(STATUS "Системну libgpiod не знайдено, вмикаємо режим компіляції для CI...")
    add_library(gpiod_mock STATIC)
    # Створюємо пустий таргет, щоб робота збудувалася в офлайн-режимі на сервері
    target_compile_definitions(gpiod_mock PUBLIC GPIOD_MOCK_BUILD)
    set(GPIOD_LIBRARY gpiod_mock)
endif()

file(GLOB_RECURSE SOURCES "src/*.cpp")

add_executable(student ${SOURCES})

target_include_directories(
    student PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include 
                   ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

target_link_libraries(student PRIVATE Threads::Threads ${GPIOD_LIBRARY})
