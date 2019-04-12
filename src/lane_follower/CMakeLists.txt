cmake_minimum_required(VERSION 2.8.3)
project(lane_follower)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  autopilot
)
find_package(OpenCV REQUIRED)
#find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs autopilot
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
#    ${EIGEN3_INCLUDE_DIRS}
)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 1)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

file(GLOB_RECURSE NODE_SOURCE RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} 
    FOLLOW_SYMLINKS src/*.cpp)
file(GLOB_RECURSE NODE_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} 
    FOLLOW_SYMLINKS include/${PROJECT_NAME}/*.hpp)

add_executable(${PROJECT_NAME} 
    ${NODE_SOURCE}
    ${NODE_MOC}
    )

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
#    ${EIGEN3_LIBRARIES}
    )
