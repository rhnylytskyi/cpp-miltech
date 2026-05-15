cmake_minimum_required(VERSION 3.20)

project(ballistics CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(ballistics_lib src/ballistics.cpp)
target_include_directories(ballistics_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(ballistics_cli src/main.cpp)
target_link_libraries(ballistics_cli PRIVATE ballistics_lib)
target_compile_options(ballistics_cli PRIVATE -Wall -Wextra -pedantic)

add_custom_target(
    run_ballistics_cli
    COMMAND ballistics_cli ${CMAKE_CURRENT_SOURCE_DIR}/data/sample_vog17.txt
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    DEPENDS ballistics_cli
    COMMENT "Running ballistics_cli with homework_06/data/sample_vog17.txt...")

if(NOT CMAKE_CROSSCOMPILING)
    include(FetchContent)
    include(GoogleTest)

    FetchContent_Declare(
        googletest URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
                       DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    set(gtest_force_shared_crt
        ON
        CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    add_executable(ballistics_tests tests/ballistics_tests.cpp)

    target_link_libraries(ballistics_tests PRIVATE ballistics_lib gtest gtest_main)

    gtest_discover_tests(ballistics_tests DISCOVERY_MODE POST_BUILD)
endif()
