cmake_minimum_required(VERSION 3.0.2)
project(network)
add_definitions(-std=c++11 -Wall -O2 -Wextra -Wno-unused-result) #-Wfloat-equal -Weffc++ -pedantic


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED roscpp)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS ${Boost_INCLUDE_DIRS}
  LIBRARIES ${Boost_LIBRARIES}
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

target_link_libraries(network_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})





