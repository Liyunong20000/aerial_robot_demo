cmake_minimum_required(VERSION 3.0.2)
project(chimera_bricks)

add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dragon
)

# find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES chimera_bricks
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



