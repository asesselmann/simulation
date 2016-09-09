cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(include/)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(muscle_plugin SHARED src/IActuator.cc src/IMuscle.cc src/ISEE.cc src/MusclePlugin.cc src/StateMachine.cc)
target_link_libraries(muscle_plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${roscpp_LIBRARIES})
