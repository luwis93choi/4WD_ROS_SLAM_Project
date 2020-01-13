cmake_minimum_required(VERSION 2.8.3)
project(slam_proto_v1)

# Add this if catkin cannot find vision_opencv or OpenCV Directory
set(OpenCV_DIR /usr/share/OpenCV)

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  rospy
  message_generation
  sensor_msgs
  std_msgs
  std_srvs
  message_filters

  # usb_cam specific requirements #
  image_transport
  camera_info_manager

  # OpenCv specific requirements #
  OpenCV
  cv_bridge

  # Transformation related requirement #
  tf

  # Navigation & Odometry related requirement #
  nav_msgs
)

add_message_files(FILES DC_velocity.msg)

generate_messages(DEPENDENCIES std_msgs)


# pkg-config libraries --> Image codec for usb_cam package #
find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)

# OpenCV requirement #
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

# usb_cam related #
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${avcodec_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# cam_node executable declaration
add_executable(cam_node src/cam_node.cpp)
add_dependencies(
  cam_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
           ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  cam_node ${catkin_LIBRARIES}
           ${avcodec_LIBRARIES}
           ${swscale_LIBRARIES}
           ${OpenCV_LIBRARIES}
)

# lidar_node executable declaration
add_executable(lidar_node src/lidar_node.cpp)
add_dependencies(
  lidar_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
             ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  lidar_node ${catkin_LIBRARIES}
)

# serial_RX_node executable declaration
add_executable(serial_RX_node src/serial_RX_node.cpp)
add_dependencies(
  serial_RX_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS}
) 
target_link_libraries(
  serial_RX_node ${catkin_LIBRARIES}
)

# transformation executable declaration
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(
  tf_broadcaster ${catkin_LIBRARIES}
)

# ackermann_odom_broadcaster executable declaration
add_executable(ackermann_odom_broadcaster src/ackermann_odom_broadcaster.cpp)
add_dependencies(
  ackermann_odom_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS}
                             ${catkin_EXPORTED_TARGETS}
) 
target_link_libraries(
  ackermann_odom_broadcaster ${catkin_LIBRARIES}
)