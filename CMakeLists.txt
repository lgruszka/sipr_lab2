cmake_minimum_required(VERSION 2.8.3)
project(sipr_lab2)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  roscpp
  std_msgs
  tf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sipr_lab2_node 
    src/sipr_lab2_node.cpp
    src/sipr_lab2/RobotController.cpp
)

target_link_libraries(sipr_lab2_node   
    ${catkin_LIBRARIES}
)
