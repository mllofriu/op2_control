cmake_minimum_required(VERSION 2.8.3)
project(robotis_op_ros_control)

# detect architecture
#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib_msgs
  actionlib
  std_msgs
  hardware_interface
  controller_manager
  dynamic_reconfigure
  tf
  message_generation
  pluginlib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# Darwin-OP interface library

SET(DARWIN_LIB_PATH /robotis/)
find_package(darwin_lib REQUIRED)
#MESSAGE(STATUS "${darwin_lib_INCLUDE_DIR}")


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   PIDPosVelAcc.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


generate_dynamic_reconfigure_options( config/robotis_op_ros_control.cfg)

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
  INCLUDE_DIRS include
  LIBRARIES robotis_op_ros_control
  CATKIN_DEPENDS roscpp rospy actionlib_msgs actionlib std_msgs hardware_interface controller_manager dynamic_reconfigure  tf message_runtime pluginlib
  DEPENDS darwin_lib_LIBRARY
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${darwin_lib_INCLUDE_DIR})

add_library(darwin_joint_controller src/darwin_joint_controller.cpp)
target_link_libraries(darwin_joint_controller ${catkin_LIBRARIES} ${darwin_lib_LIBRARY})
add_dependencies(darwin_joint_controller ${PROJECT_NAME}_msgs_generate_messages_cpp)

## Declare a cpp executable
add_executable(${PROJECT_NAME} src/robotis_op_ros_controller.cpp src/robotis_op_hardware_interface.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_msgs_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${darwin_lib_LIBRARY})

add_executable(move tests/move.cpp)
#add_dependencies(move ${PROJECT_NAME}_msgs_generate_messages_cpp)
target_link_libraries(move ${catkin_LIBRARIES} ${darwin_lib_LIBRARY})

## Specify libraries to link a library or executable target against

#target_link_libraries(robotis_op_ros_controller ${PROJECT_NAME})


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
 
install(TARGETS darwin_joint_controller ${PROJECT_NAME} move
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
