cmake_minimum_required(VERSION 3.0.2)
project(read_keyboard)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES read_keyboard
#  CATKIN_DEPENDS rospy rpscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
  src/read_keyboard.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

