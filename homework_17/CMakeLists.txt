cmake_minimum_required(VERSION 3.20)
project(homework_17)

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

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(homework_17 ${SOURCES})

target_include_directories(homework_17 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
                                               ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

target_link_libraries(homework_17 PRIVATE Threads::Threads ${GPIOD_LIBRARY})

target_compile_definitions(homework_17
                           PRIVATE PROJECT_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
