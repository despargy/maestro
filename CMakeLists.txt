cmake_minimum_required(VERSION 3.0.2)
project(maestro)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -lorocos-kdl)

## Find catkin macros and libraries
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
  genmsg
  gazebo_msgs
  eigen_conversions
)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

generate_messages(
  DEPENDENCIES 
  std_msgs
  geometry_msgs
  sensor_msgs
  unitree_legged_msgs
)

catkin_package(
 INCLUDE_DIRS include 
 LIBRARIES maestro
 CATKIN_DEPENDS roscpp rospy message_runtime sensor_msgs std_msgs gazebo_msgs geometry_msgs unitree_legged_msgs kdl_parser robot_state_publisher joint_state_controller
 DEPENDS system_lib  
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include   ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR}
  ${unitree_legged_msgs_INCLUDE_DIRS}
  )
link_directories($(catkin_LIB_DIRS) lib)

SET( rcd_cpp_src
  src/Robot.cpp
  src/CommunicationHandler.cpp
  src/Controller.cpp
  src/Leg.cpp
  src/Math.cpp
  src/DataHandler.cpp
)

add_executable(main_handler src/MainHandler.cpp ${rcd_cpp_src}) 
add_dependencies(main_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages unitree_legged_msgs_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(main_handler   ${catkin_LIBRARIES})

add_executable(maestro_gazebo_interpreter src/GazeboInterpreter.cpp ${rcd_cpp_src}) 
add_dependencies(maestro_gazebo_interpreter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages unitree_legged_msgs_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(maestro_gazebo_interpreter   ${catkin_LIBRARIES})

