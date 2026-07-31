cmake_minimum_required(VERSION 3.20)
project(BallisticAutopilot)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Threads REQUIRED)

# Look for the system C-library gpiod
find_library(GPIOD_LIBRARY NAMES gpiod)
set(GPIOD_INCLUDE_DIR "")

if(NOT GPIOD_LIBRARY)
    message(STATUS "System libgpiod not found, enabling CI mock build mode...")

    add_library(gpiod_mock INTERFACE)
    target_compile_definitions(gpiod_mock INTERFACE GPIOD_MOCK_BUILD)
    target_include_directories(gpiod_mock INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include/mock)

    set(GPIOD_LIBRARY gpiod_mock)
else()
    message(STATUS "Found real system gpiod library: ${GPIOD_LIBRARY}")
endif()

# Automatically collect all source files from the src directory
file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(drone_autopilot ${SOURCES})

# Core project include directories
target_include_directories(drone_autopilot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
                                                  ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

# Link threads interface and selected gpiod library target
target_link_libraries(drone_autopilot PRIVATE Threads::Threads ${GPIOD_LIBRARY})

# Preprocessor definitions
target_compile_definitions(drone_autopilot
                           PRIVATE PROJECT_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
