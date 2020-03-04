cmake_minimum_required(VERSION 2.8.3)
project(ros_rplidar_practice)

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  sensor_msgs
  std_srvs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS sensor_msgs roscpp
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(process_node src/process_node.cpp)
add_dependencies(
  process_node ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  process_node 
  ${catkin_LIBRARIES}
)
