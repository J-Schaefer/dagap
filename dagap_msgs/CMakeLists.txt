cmake_minimum_required(VERSION 3.0.2)
project(dagap_msgs)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(MSG_DEPS
  std_msgs
  geometry_msgs
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${MSG_DEPS}
  message_generation
  rospy
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
set(MSG_FILES
   FILES
   OPMObjectQuery.msg
)

## Generate services in the 'srv' folder
set(SRV_FILES
   FILES
   GetGraspPose.srv
   Query.srv
)

add_message_files(DIRECTORY msg FILES ${MSG_FILES})
add_service_files(DIRECTORY srv FILES ${SRV_FILES})


# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES ${MSG_DEPS})


###################################
## catkin specific configuration ##
###################################
catkin_package(DEPENDS ${MSG_DEPS} message_runtime)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
