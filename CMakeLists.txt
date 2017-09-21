project(slidewindow)

cmake_minimum_required(VERSION 3.9)

find_package(OpenCV)

add_executable(slidewindow slidewindow_main.cpp)

target_link_libraries(slidewindow ${OpenCV_LIBS})
