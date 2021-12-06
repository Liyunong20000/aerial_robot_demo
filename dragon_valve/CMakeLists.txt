cmake_minimum_required(VERSION 3.0.2)
project(dragon_valve)

find_package(NLopt REQUIRED)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dragon
)

find_package(OpenMP)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

catkin_python_setup()

catkin_package(
  #  INCLUDE_DIRS include
#  LIBRARIES dragon_valve
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
)

add_executable(max_torque src/max_torque.cpp)
target_link_libraries (max_torque ${catkin_LIBRARIES} ${NLOPT_LIBRARIES})

