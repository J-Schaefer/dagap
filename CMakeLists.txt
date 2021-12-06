cmake_minimum_required(VERSION 2.8.3)
project(dagap)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(MSG_DEPS
  std_msgs
  geometry_msgs
)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${MSG_DEPS}
  rospy
  message_generation
)


## Generate services in the 'srv' folder
set(SRV_FILES
   FILES
   GetGraspPose.srv
 )

add_service_files(DIRECTORY srv FILES ${SRV_FILES})

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   ${MSG_DEPS}
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
#  LIBRARIES dagap
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 catkin_install_python(PROGRAMS
   scripts/dagap_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
