cmake_minimum_required(VERSION 2.8.3)
project(pick_up_exekutor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  acquire_exekutor
  cam_interface
  action_exekutor
  look_exekutor
  moveit_hand_exekutor
  tf
  tuples_linker
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pick_up_exekutor handover_exekutor
  CATKIN_DEPENDS acquire_exekutor action_exekutor look_exekutor moveit_hand_exekutor tf
)

##############################
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
##############################

add_library(pick_up_exekutor
  src/pick_up_exekutor.cpp
)

add_dependencies(pick_up_exekutor doro_manipulation_generate_messages_cpp)

target_link_libraries(pick_up_exekutor
    ${catkin_LIBRARIES}
    ${tuples_linker_LIBRARIES}
)

##############################

add_executable(pick_up_exekutor_test src/pick_up_exekutor_test.cc)

target_link_libraries(pick_up_exekutor_test
  ${catkin_LIBRARIES}
  pick_up_exekutor
  peiskernel_mt
)

##############################

add_library(handover_exekutor src/handover_exekutor.cpp)

add_dependencies (handover_exekutor doro_manipulation_generate_messages_cpp)

target_link_libraries(handover_exekutor
    ${catkin_LIBRARIES}
)

##############################

add_executable (handover_exekutor_test src/handover_exekutor_test.cpp)

target_link_libraries(handover_exekutor_test
    handover_exekutor
    peiskernel_mt
    ${catkin_LIBRARIES}
)

##############################