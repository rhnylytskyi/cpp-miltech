cmake_minimum_required(VERSION 3.20)
project(BallisticAutopilot)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Threads REQUIRED)

# Шукаємо саме C-бібліотеку gpiod
find_library(GPIOD_LIBRARY NAMES gpiod)

# Створюємо змінну для додаткових шляхів інклудів
set(GPIOD_INCLUDE_DIR "")

if(NOT GPIOD_LIBRARY)
    message(STATUS "Системну libgpiod не знайдено, вмикаємо режим компіляції для CI...")

    add_library(gpiod_mock INTERFACE)
    target_compile_definitions(gpiod_mock INTERFACE GPIOD_MOCK_BUILD)

    # Тільки для CI додаємо шлях до папки, де тепер захована заглушка:
    target_include_directories(gpiod_mock INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include/mock)

    set(GPIOD_LIBRARY gpiod_mock)
else()
    message(STATUS "Знайдено реальну системну бібліотеку: ${GPIOD_LIBRARY}")
endif()

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(student ${SOURCES})

# Основні інклуди проекту
target_include_directories(student PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
                                          ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

# Лінкуємо Threads та gpiod (реальну або mock, яка сама підтягне свій include/mock якщо треба)
target_link_libraries(student PRIVATE Threads::Threads ${GPIOD_LIBRARY})

target_compile_definitions(student PRIVATE PROJECT_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
