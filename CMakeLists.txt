cmake_minimum_required(VERSION 2.8.3)
project(fetch_demos)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  actionlib_msgs
  actionlib
)
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_executable(${PROJECT_NAME}_follow src/follow.cpp)

# target_link_libraries(${PROJECT_NAME}_follow
#   ${catkin_LIBRARIES}
# )
