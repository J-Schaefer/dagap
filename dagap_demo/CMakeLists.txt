
cmake_minimum_required(VERSION 3.0.2)
project(dagap_demo)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib_lisp
  actionlib_msgs
  dagap_msgs
  roslisp
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 CATKIN_DEPENDS actionlib_lisp actionlib_msgs dagap_msgs roslisp std_msgs
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
