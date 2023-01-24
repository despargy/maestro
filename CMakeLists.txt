cmake_minimum_required(VERSION 3.0.2)
project(maestro)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
  unitree_legged_msgs
  urdf
  kdl_parser
  joint_state_controller
  robot_state_publisher
)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  LowStateROS.msg
  LowCmdROS.msg
  Foot.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
 INCLUDE_DIRS include 
 LIBRARIES maestro
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs geometry_msgs unitree_legged_msgs kdl_parser robot_state_publisher joint_state_controller
 DEPENDS system_lib
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /home/despargy/go1_ws/src/unitree_legged_sdk/include
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/MainHandler.cpp
# )

SET( rcd_cpp_src
  src/Robot.cpp
  src/CommunicationHandler.cpp
  src/Controller.cpp
)



add_executable(main_handler src/MainHandler.cpp ${rcd_cpp_src}) 
add_dependencies(main_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main_handler   ${catkin_LIBRARIES} )
