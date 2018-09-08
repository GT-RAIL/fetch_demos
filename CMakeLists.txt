cmake_minimum_required(VERSION 2.8.3)
project(fetch_demos)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  roscpp
  rospy
  moveit_python
  tf
)

catkin_package(
  CATKIN_DEPENDS
  actionlib
  roscpp
  rospy
  moveit_python
  tf
)

# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )

# add_executable(${PROJECT_NAME}_follow src/follow.cpp)

# target_link_libraries(${PROJECT_NAME}_follow
#   ${catkin_LIBRARIES}
# )
