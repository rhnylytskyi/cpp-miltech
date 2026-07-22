cmake_minimum_required(VERSION 3.20)
project(BallisticAutopilot)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Threads REQUIRED)

find_library(GPIOD_LIBRARY NAMES gpiod)
find_path(GPIOD_INCLUDE_DIR NAMES gpiod.h)

if(NOT GPIOD_LIBRARY OR NOT GPIOD_INCLUDE_DIR)
    message(
        FATAL_ERROR "libgpiod не знайдено! Виконайте в терміналі: sudo apt install libgpiod-dev")
endif()

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(ballistic_app_11 ${SOURCES})

target_include_directories(ballistic_app_11 PUBLIC include ${GPIOD_INCLUDE_DIR})
target_link_libraries(ballistic_app_11 PRIVATE Threads::Threads ${GPIOD_LIBRARY})
