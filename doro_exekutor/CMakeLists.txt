cmake_minimum_required(VERSION 2.8.3)
project(doro_exekutor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  acquire_exekutor
  move_to_exekutor
  action_exekutor
  look_exekutor
  move_hand_exekutor
  moveit_hand_exekutor
  pick_up_exekutor
  dock_exekutor
  tuples_linker
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES exekutor_executable
  CATKIN_DEPENDS acquire_exekutor action_exekutor look_exekutor moveit_hand_exekutor pick_up_exekutor
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(doro_exekutor src/exekutor_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(doro_exekutor
  ${catkin_LIBRARIES}
  peiskernel_mt
)
