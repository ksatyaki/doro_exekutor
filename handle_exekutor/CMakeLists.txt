cmake_minimum_required(VERSION 2.8.3)
project(handle_exekutor)

find_package(catkin REQUIRED COMPONENTS
  action_exekutor
  led_handle_board
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES handle_exekutor
  CATKIN_DEPENDS action_exekutor led_handle_board
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(handle_exekutor
  src/handle_exekutor.cpp
)
target_link_libraries(handle_exekutor ${catkin_LIBRARIES})
## Declare a cpp executable
# add_executable(handle_exekutor_node src/handle_exekutor_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(handle_exekutor_node handle_exekutor_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(handle_exekutor_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS handle_exekutor handle_exekutor_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_handle_exekutor.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
