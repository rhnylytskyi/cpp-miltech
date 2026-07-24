cmake_minimum_required(VERSION 3.20)
project(BallisticAutopilot)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Threads REQUIRED)

find_library(GPIOD_LIBRARY NAMES gpiod)
find_path(GPIOD_INCLUDE_DIR NAMES gpiod.h)

if(NOT GPIOD_LIBRARY OR NOT GPIOD_INCLUDE_DIR)
    message(
        STATUS
            "Системну libgpiod не знайдено! Автоматично завантажуємо її з офіційного репозиторію..."
    )

    include(FetchContent)
    FetchContent_Declare(
        libgpiod
        GIT_REPOSITORY https://kernel.org
        GIT_TAG v1.6.3)

    set(gpiod_build_tools
        OFF
        CACHE INTERNAL "")
    set(gpiod_build_tests
        OFF
        CACHE INTERNAL "")

    FetchContent_MakeAvailable(libgpiod)

    set(GPIOD_LIBRARY gpiod)
    set(GPIOD_INCLUDE_DIR ${libgpiod_SOURCE_DIR}/include)
endif()

file(GLOB_RECURSE SOURCES "src/*.cpp")

add_executable(student ${SOURCES})

target_include_directories(
    student PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/third_party
                   ${GPIOD_INCLUDE_DIR})

target_link_libraries(student PRIVATE Threads::Threads ${GPIOD_LIBRARY})
