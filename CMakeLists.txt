cmake_minimum_required(VERSION 3.0.2)
project(px4_test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    tf
    message_generation
    message_runtime
    mavros_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  state.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  pose.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  mavros_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES computer_vision
#  CATKIN_DEPENDS other_catkin_pkg
 DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/computer_vision.cpp
# )

add_executable(state src/state.cpp)
add_executable(offb src/offb.cpp)

add_dependencies(state ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(offb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(state
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)
target_link_libraries(offb
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)