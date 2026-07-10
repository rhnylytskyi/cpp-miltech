cmake_minimum_required(VERSION 3.20)

project(ballistic_app_07 CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(ballistic_app_07 ${SOURCES})

target_include_directories(ballistic_app_07 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(ballistic_app_07 PRIVATE nlohmann_json::nlohmann_json)

target_compile_options(ballistic_app_07 PRIVATE -Wall -Wextra -pedantic)

target_compile_definitions(
    ballistic_app_07 PRIVATE CONFIG_PATH="data/config.json" AMMO_PATH="data/ammo.json"
                          TARGETS_PATH="data/targets.json" SIMULATION_PATH="data/simulation.json")
