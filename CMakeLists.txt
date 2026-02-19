cmake_minimum_required(VERSION 3.10)
project(TestProject)

set(CMAKE_CXX_STANDARD 26)

find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

set(SOURCES src/main.cpp src/display.cpp)

add_executable(test ${SOURCES})
target_link_libraries(test ${SDL2_LIBRARIES})