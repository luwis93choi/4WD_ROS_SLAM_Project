cmake_minimum_required(VERSION 2.8.3)
project(ros_usb_cam_practice)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs

  # usb_cam specific required packages
  image_transport
  std_srvs
  sensor_msgs
  camera_info_manager

  # OpenCV specific required packages
  OpenCV
  cv_bridge
)

## pkg-config libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)

## OpenCV requirement
find_package(OpenCV REQUIRED)

## Declare messages
add_message_files(FILES main_node_msg.msg)

generate_messages(DEPENDENCIES std_msgs)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########


#### usb_cam related ###################################################
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${avcodec_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/usb_cam.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
)

add_executable(${PROJECT_NAME}_CAMnode nodes/usb_cam_node.cpp)
target_link_libraries(
  ${PROJECT_NAME}_CAMnode
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
)
################################################################################

#### OpenCV related ############################################################
include_directories(${OpenCV_INCLUDE_DIRS})
################################################################################

add_executable(main_node nodes/main_node.cpp)
add_dependencies(
  main_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(main_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}) # OpenCV Library path is included since main_node.cpp will use OpenCV library
