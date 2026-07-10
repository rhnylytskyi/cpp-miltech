cmake_minimum_required(VERSION 3.20)

project(ballistics CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(ballistics src/main.cpp)

target_compile_definitions(ballistics PRIVATE 
    DATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data/input.txt"
    TARGETS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data/targets.txt"
    SIMULATION_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data/simulation.txt"
    DEBUG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data/debug.txt"
)

target_compile_options(ballistics PRIVATE /W4 /permissive-)