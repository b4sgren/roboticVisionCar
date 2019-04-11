cmake_minimum_required(VERSION 3.4)
project(FinalProject)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(follower follower.cpp)
target_link_libraries(follower ${OpenCV_LIBS})
