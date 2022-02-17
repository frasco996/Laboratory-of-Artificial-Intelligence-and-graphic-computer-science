cmake_minimum_required(VERSION 3.0.2)
project(Obstacle_avoidance)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  laser_geometry
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
 )

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES Obstacle_avoidance
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_Avoidance src/robot_Avoidance.cpp)
target_link_libraries(robot_Avoidance ${catkin_LIBRARIES})
add_dependencies(robot_Avoidance Obstacle_avoidance_generate_messages_cpp)
