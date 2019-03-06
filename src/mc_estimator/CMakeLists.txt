cmake_minimum_required(VERSION 2.8.3)
project(mc_estimator)

set(CMAKE_CXX_FLAGS "-fopenmp")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  cv_bridge
  message_generation
  autopilot
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# add_service_files(
  # FILES
#   Drc.srv
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(estimator_node   src/mc_estimator.cpp   include/mc_estimator.h)
target_link_libraries(estimator_node ${catkin_LIBRARIES})
add_dependencies(estimator_node ${catkin_EXPORTED_TARGETS})

# add_executable(collector_node  src/collector.cpp   include/collector.h)
# target_link_libraries(collector_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
