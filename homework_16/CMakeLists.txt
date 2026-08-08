cmake_minimum_required(VERSION 3.20)
project(homework_16 CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CheckIncludeFileCXX)
check_include_file_cxx("linux/i2c-dev.h" HAVE_LINUX_I2C_DEV_H)

if(NOT HAVE_LINUX_I2C_DEV_H)
    message(
        WARNING "linux/i2c-dev.h not found (Linux/WSL/Devcontainer is required) — Target skipped")
else()
    file(GLOB_RECURSE SOURCES "src/*.cpp")
    add_executable(homework_16 ${SOURCES})

    target_include_directories(homework_16 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_compile_options(homework_16 PRIVATE -Wall -Wextra -pedantic)
endif()

# Automatically copy the simulator library to the build directory for convenience
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/libi2csim.so ${CMAKE_CURRENT_BINARY_DIR}/libi2csim.so
               COPYONLY)
