cmake_minimum_required(VERSION 3.5)

project(voir)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS})

add_executable(detection main.cpp)

target_link_libraries(detection ${OpenCV_LIBS})
