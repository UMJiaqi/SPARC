cmake_minimum_required(VERSION 3.0.2)
project(controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES controller
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
  src/main.cpp 
  src/gen_path.cpp 
  src/pure_pursuit.cpp 
  src/keyboard_controller.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
