cmake_minimum_required(VERSION 3.16)
project(homework_17 CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Search for mandatory system packages (Threads and libgpiod)
find_package(Threads REQUIRED)
find_library(GPIOD_LIB gpiod REQUIRED)

# Configure system-wide search header target directories
include_directories(include third_party)

# Gather all executable target implementation source files recursively
file(GLOB_RECURSE SRC_FILES "src/*.cpp")

# Declare the final compiled vehicle autopilot binary file
add_executable(homework_17 ${SRC_FILES})

# Inject absolute runtime storage path macros for ballistic tables
target_compile_definitions(homework_17 PRIVATE PROJECT_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

# Bind hardware libraries and thread packages to the application target
target_link_libraries(homework_17 PRIVATE Threads::Threads ${GPIOD_LIB})
