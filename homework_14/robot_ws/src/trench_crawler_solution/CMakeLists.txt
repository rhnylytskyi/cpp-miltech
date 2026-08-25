cmake_minimum_required(VERSION 3.20)
project(trench_crawler_solution)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(underground_world REQUIRED)

add_executable(trench_navigator src/trench_navigator.cpp)
ament_target_dependencies(trench_navigator rclcpp underground_world)

add_executable(weapon_action src/weapon_action.cpp)
ament_target_dependencies(weapon_action rclcpp underground_world)

install(TARGETS trench_navigator weapon_action DESTINATION lib/${PROJECT_NAME})

ament_package()
