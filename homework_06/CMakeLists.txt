cmake_minimum_required(VERSION 3.20)

project(ballistics CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(ballistics_cli src/main.cpp)

add_library(ballistics src/ballistics.cpp)
target_include_directories(ballistics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(ballistics_cli PRIVATE ballistics)

target_compile_options(ballistics_cli PRIVATE -Wall -Wextra -pedantic)

add_custom_target(run_ballistics_cli
    # 1. Запускаємо скомпільований файл ballistics_cli
    # 2. Передаємо йому шлях до sample_vog17.txt, який лежить у data/ поруч із цим CMakeLists
    COMMAND ballistics_cli ${CMAKE_CURRENT_SOURCE_DIR}/data/sample_vog17.txt
    
    # Робоча директорія запуску — папка homework_06
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    
    # Вказуємо залежність від виконуваного файлу
    DEPENDS ballistics_cli
    
    COMMENT "Running ballistics_cli with homework_06/data/sample_vog17.txt..."
)