# Copyright (c) Synexens Corporation. All rights reserved.

cmake_minimum_required(VERSION 3.5)
project(synexens_ros_driver LANGUAGES C CXX)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  image_transport
  image_geometry
  nodelet
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nodelet
  cv_bridge
  camera_info_manager
)

find_package(OpenCV REQUIRED)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES k4a_ros_bridge
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

############################
#### Synexens3 SDK ######
############################

message("Finding Synexens3 SDK binaries")

set(OpenCV_LIBS opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_videoio opencv_calib3d)
set(SYN_LIBS csreconstruction SonixCamera synexens3 ${OpenCV_LIBS})
set(SYN_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/ext/sdk)
set(SYN_INCLUDE_DIR ${SYN_SDK_PATH}/include)
set(SYN_LIB_DIR ${SYN_SDK_PATH}/lib)
set(OPENCV_LIB_DIR ${SYN_SDK_PATH}/opencv)

link_directories(
  ${SYN_LIB_DIR}
  ${OPENCV_LIB_DIR}
)

##################################
###### END Synexens3 SDK ######
##################################

###########
## Build ##
###########


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node
  src/synexens_ros_bridge_node.cpp
  src/synexens_ros_device.cpp
  src/synexens_ros_device_params.cpp
  src/synexens_calibration_transform_data.cpp
)

target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_11)

add_library(${PROJECT_NAME}_nodelet
  src/synexens_ros_bridge_nodelet.cpp
  src/synexens_ros_device.cpp
  src/synexens_ros_device_params.cpp
  src/synexens_calibration_transform_data.cpp
)

target_compile_features(${PROJECT_NAME}_nodelet PUBLIC cxx_std_11)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${SYN_INCLUDE_DIR}
  "include"
)

target_link_libraries(${PROJECT_NAME}_node
  ${SYN_LIBS}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${SYN_LIBS}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY
  launch
  rviz
  scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  ${SYN_LIB_DIR}/
  ${OPENCV_LIB_DIR}/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
   nodelet_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
