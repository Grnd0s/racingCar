cmake_minimum_required(VERSION 3.5)

project(voir)

find_package(raspicam REQUIRED)
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS})

add_executable(raspi_car main.cpp)

target_link_libraries(raspi_car ${OpenCV_LIBS} ${raspicam_CV_LIBS})
