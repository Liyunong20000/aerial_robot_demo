cmake_minimum_required(VERSION 3.0.2)
project(dragon_valve)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_python_setup()


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dragon_valve
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
