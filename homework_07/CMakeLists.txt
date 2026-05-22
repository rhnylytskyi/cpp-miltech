cmake_minimum_required(VERSION 3.20)

project(BallisticApp CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(ballistic_app ${SOURCES})

target_include_directories(ballistic_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(ballistic_app PRIVATE nlohmann_json::nlohmann_json)

target_compile_options(ballistic_app PRIVATE -Wall -Wextra -pedantic)

target_compile_definitions(
    ballistic_app PRIVATE CONFIG_PATH="data/config.json" AMMO_PATH="data/ammo.json"
                          TARGETS_PATH="data/targets.json" SIMULATION_PATH="data/simulation.json")
