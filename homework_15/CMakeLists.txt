cmake_minimum_required(VERSION 3.20)

project(ballistic_app_15 CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(ballistic_app_15 ${SOURCES})

target_include_directories(ballistic_app_15 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(ballistic_app_15 PRIVATE nlohmann_json::nlohmann_json)

target_compile_options(ballistic_app_15 PRIVATE -Wall -Wextra -pedantic)

include(FetchContent)
FetchContent_Declare(
    httplib
    GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
    GIT_TAG v0.18.3
    GIT_SHALLOW TRUE)

set(HTTPLIB_REQUIRE_OPENSSL
    OFF
    CACHE BOOL "" FORCE)
set(HTTPLIB_REQUIRE_ZLIB
    OFF
    CACHE BOOL "" FORCE)
set(HTTPLIB_REQUIRE_BROTLI
    OFF
    CACHE BOOL "" FORCE)
set(HTTPLIB_COMPILE
    OFF
    CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(httplib)

target_link_libraries(ballistic_app_15 PRIVATE httplib::httplib)
