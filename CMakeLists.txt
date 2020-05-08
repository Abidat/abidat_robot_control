cmake_minimum_required(VERSION 2.8.3)
project(abidat_robot_control)

##Compile with C++11 Standards
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  message_generation
  genmsg 
)

## Generate messages.
add_message_files(
  FILES
  DeviceInfo.msg
  MotorState.msg
  MotorControl.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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
#  LIBRARIES
  CATKIN_DEPENDS
    message_runtime
    roscpp
    geometry_msgs
    std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  src
  src/library
)

## Declare a C++ library
add_library(abidat_robot_control SHARED               
  src/library/brick_pi/BrickPi3.cpp
  src/library/input_mapping.cpp
  src/library/get_keyboard_input.cpp
)

## Declare a C++ executable
add_executable(motor_handler
  src/nodes/motor_handler_node.cpp
)

add_executable(forward_kinematic
  src/nodes/forward_kinematic_node.cpp
)

add_executable(teleoperation
  src/nodes/teleoperation_node.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(abidat_robot_control
  ${catkin_LIBRARIES}
  ncurses # \Todo use "find package" instead
)

target_link_libraries(motor_handler
  abidat_robot_control
  ${catkin_LIBRARIES}
)

target_link_libraries(forward_kinematic
  abidat_robot_control
  ${catkin_LIBRARIES}
)

target_link_libraries(teleoperation
  abidat_robot_control
  ${catkin_LIBRARIES}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies( abidat_robot_control ${PROJECT_NAME}_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(motor_handler ${PROJECT_NAME}_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(teleoperation ${PROJECT_NAME}_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(forward_kinematic ${PROJECT_NAME}_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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
#install(TARGETS
#  motor_handler
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

#install(DIRECTORY
#  models
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#install(DIRECTORY launch/
#  DESTINATION
#  ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#  FILES_MATCHING PATTERN "*.launch"
#)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_nrbdl_robots.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

find_package(GTest REQUIRED)
find_package(rostest REQUIRED)

#ROS TESTS
add_rostest_gtest(rostest_teleop_node
  test/launch/rostest_teleop_node.test
  test/src/rostest_teleop_node.cpp 
)

target_link_libraries(rostest_teleop_node
  abidat_robot_control
)

#GTESTS
#input_mapper
catkin_add_gtest(test_input_mapper 
  test/src/test_input_mapper.cpp
)

if(TARGET test_input_mapper)
  target_link_libraries(test_input_mapper
    abidat_robot_control
  )
endif()

#get_keyboard_input
# catkin_add_gtest(test_get_keyboard_input 
#   test/src/test_get_keyboard_input.cpp
# )

# if(TARGET test_get_keyboard_input)
#   target_link_libraries(test_get_keyboard_input
#     abidat_robot_control
#   )
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
