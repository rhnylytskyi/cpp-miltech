cmake_minimum_required(VERSION 3.20)

project(ballistic_app_09 CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(ballistic_app_09 ${SOURCES})

target_include_directories(ballistic_app_09 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(ballistic_app_09 PRIVATE nlohmann_json::nlohmann_json)

target_compile_options(ballistic_app_09 PRIVATE -Wall -Wextra -pedantic)