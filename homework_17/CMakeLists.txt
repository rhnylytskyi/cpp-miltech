cmake_minimum_required(VERSION 3.20)
project(homework_17 CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Search for mandatory system packages (Threads and libgpiod)
find_package(Threads REQUIRED)
find_library(GPIOD_LIBRARY NAMES gpiod)

# Fallback block from option 1 to handle environment tests without real hardware
if(NOT GPIOD_LIBRARY)
    message(STATUS "System libgpiod not found, enabling CI mock build mode...")
    add_library(gpiod_mock INTERFACE)
    target_compile_definitions(gpiod_mock INTERFACE GPIOD_MOCK_BUILD)
    target_include_directories(gpiod_mock INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include/mock)
    set(GPIOD_LIBRARY gpiod_mock)
else()
    message(STATUS "Found real system gpiod library: ${GPIOD_LIBRARY}")
endif()

# Gather all executable target implementation source files recursively
file(GLOB_RECURSE SRC_FILES "src/*.cpp")

# Declare the final compiled vehicle autopilot binary file
add_executable(homework_17 ${SRC_FILES})

# Configure search header target directories including the custom MAVLink v2 repository c_library_v2
target_include_directories(homework_17 SYSTEM
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/c_library_v2)
target_include_directories(homework_17 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
                                              ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

# Inject absolute runtime storage path macros for ballistic tables
target_compile_definitions(homework_17 PRIVATE PROJECT_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

# Bind hardware libraries and thread packages to the application target
target_link_libraries(homework_17 PRIVATE Threads::Threads ${GPIOD_LIBRARY})
