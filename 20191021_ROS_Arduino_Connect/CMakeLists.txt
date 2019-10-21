cmake_minimum_required(VERSION 2.8.3)
project(ros_arduino_connect)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS std_msgs roscpp
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)


### Defining serial_RX_node ######################################
add_executable(serial_RX_node src/serial_RX_node.cpp)
add_dependencies(
  serial_RX_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  serial_RX_node
  ${catkin_LIBRARIES}
)
##################################################################