cmake_minimum_required(VERSION 3.20)

# Окремий CMake-проєкт для домашньої роботи 05. Його можна збирати як
# через root CMakeLists.txt, так і напряму з цієї директорії.
project(telemetry CXX)

# Код домашньої роботи збирається з тим самим стандартом, що й лекційні приклади.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Один executable для задачі з telemetry.
add_executable(telemetry_check src/main.cpp)

# Бібліотека telemetry, яка містить код для роботи з телеметрією.
# Її можна буде використовувати в інших частинах проєкту, якщо потрібно. 
# Вона збирається з файлу src/telemetry.cpp, а її заголовочні файли знаходяться в include директорії. 
# Це дозволяє іншим частинам проєкту підключати її через #include "telemetry.hpp".
add_library(telemetry src/telemetry.cpp)
target_include_directories(telemetry PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Зв'язуємо executable з бібліотекою, щоб він міг використовувати її функціонал.
target_link_libraries(telemetry_check PRIVATE telemetry)

# Базові compiler warnings допомагають побачити підозрілі місця до runtime.
# Вони включають всі основні попередження, а також додаткові, які можуть виявити потенційні проблеми в коді.
target_compile_options(telemetry_check PRIVATE -Wall -Wextra -pedantic)
