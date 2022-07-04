cmake_minimum_required(VERSION 3.8)
project(abidat_robot_control)

##Compile with C++11 Standards
add_compile_options(-std=c++17 -O2)

# Find dependencies (ROS)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/DeviceInfo.msg
  msg/MotorState.msg
  msg/MotorControl.msg
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
  ADD_LINTER_TESTS
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Build nodes
## Build robot control C++ library
add_library(abidat_robot_control_base SHARED               
  src/library/brick_pi/BrickPi3.cpp
  src/library/input_mapping.cpp
  src/library/get_keyboard_input.cpp
)

target_link_libraries(abidat_robot_control_base
  PRIVATE ncurses # \Todo use "find package" instead
)

## Nodes
add_executable(motor_handler
  src/nodes/motor_handler_node.cpp
)
target_link_libraries(motor_handler
  abidat_robot_control_base
  "${cpp_typesupport_target}"
)
ament_target_dependencies(motor_handler
  rclcpp
)


add_executable(forward_kinematic
  src/nodes/forward_kinematic_node.cpp
)
target_link_libraries(forward_kinematic
  abidat_robot_control_base
)
ament_target_dependencies(forward_kinematic
  rclcpp
)


add_executable(teleoperation
  src/nodes/teleoperation_node.cpp
)
target_link_libraries(teleoperation
  abidat_robot_control_base
)
ament_target_dependencies(teleoperation
  rclcpp
)



#############
## Install ##
#############
# Install node(s)
install(TARGETS abidat_robot_control_base motor_handler forward_kinematic teleoperation
  DESTINATION lib/${PROJECT_NAME})

# Install all launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install all config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

## Testing node functionality
ament_add_gtest(rostest_teleop_node
  test/launch/rostest_teleop_node.test
  test/src/rostest_teleop_node.cpp 
)
if (TARGET rostest_teleop_node)
  target_link_libraries(rostest_teleop_node
    abidat_robot_control_base
  )
endif()


ament_add_gtest(rostest_forward_kinematic_node
  test/launch/rostest_forward_kinematic_node.test
  test/src/rostest_forward_kinematic_node.cpp
)
if (TARGET rostest_forward_kinematic_node)
  target_link_libraries(rostest_teleop_node
    abidat_robot_control_base
  )
endif()


## Unit tests
ament_add_gtest(test_input_mapper 
  test/src/test_input_mapper.cpp
)

if(TARGET test_input_mapper)
  target_link_libraries(test_input_mapper
    abidat_robot_control_base
  )
endif()

ament_add_gtest(test_forward_kinematics
  test/src/test_forward_kinematics.cpp
)

if(TARGET test_forward_kinematics)
  target_link_libraries(test_forward_kinematics
    abidat_robot_control_base
  )
endif()
