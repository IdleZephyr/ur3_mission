cmake_minimum_required(VERSION 3.0.2)
project(robotiq_2f_gripper_action_server)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  robotiq_2f_gripper_control
  roscpp
  control_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib actionlib_msgs robotiq_2f_gripper_control roscpp control_msgs
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${robotiq_2f_gripper_action_server_INCLUDE_DIRS}
)

# The action server
add_executable(robotiq_2f_gripper_action_server_node
    src/robotiq_2f_gripper_action_server_node.cpp
    src/robotiq_2f_gripper_action_server.cpp
    include/robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server.h)

add_dependencies(robotiq_2f_gripper_action_server_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(robotiq_2f_gripper_action_server_node
   ${catkin_LIBRARIES}
 )

 # The test server
add_executable(robotiq_2f_gripper_action_server_client_test
  src/robotiq_2f_gripper_action_server_client_test.cpp
)

add_dependencies(robotiq_2f_gripper_action_server_client_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(robotiq_2f_gripper_action_server_client_test
   ${catkin_LIBRARIES}
 )


install(TARGETS robotiq_2f_gripper_action_server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 install(TARGETS robotiq_2f_gripper_action_server_client_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
 )
