cmake_minimum_required(VERSION 2.8.3)
project(winter_project)

# add_definitions(-std=c++11) THIS WAS CAUSING SEGFAULT WITH CERTAIN PCL STUFF

# SHOULDN'T NEED THIS IF THE PROGRAM COMPILED JUST FINE...
# FIND_LIBRARY(HDF5_LIBRARY hdf5 /usr/local/hdf5/lib/)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  genmsg
  pcl_ros
  pcl_conversions
  pcl_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  velodyne_msgs
  nav_msgs
  base_local_planner
  costmap_2d
  move_base
  move_base_msgs
  actionlib
  nav_core
  tf
  nav_msgs
  pluginlib
  cmake_modules
  dynamic_reconfigure
#  std_srvs
#  cv_bridge
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# find_package(OpenCV)
#find_package(octomap REQUIRED)
find_package(PCL REQUIRED)
find_package(PkgConfig REQUIRED)
# find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)


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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#  std_srvs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  LIBRARIES winter_project
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(winter_project
#   src/${PROJECT_NAME}/winter_project.cpp
# )

add_library(custom_planner_lib src/custom_planner.cpp)
#target_link_libraries(custom_planner_lib ${catkin_LIBRARIES} base_local_planner)
target_link_libraries(custom_planner_lib ${catkin_LIBRARIES})

#add_library(button_cancel_navigation_lib src/button_cancel_navigation.cpp)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(winter_project ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(winter_project_node src/winter_project_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(winter_project_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(winter_project_node
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
# install(TARGETS winter_project winter_project_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_winter_project.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


# pkg_check_modules( ncurses++ REQUIRED ncurses++ )
# add_executable(keyboard_control src/keyboard_control.cpp)
# target_link_libraries(keyboard_control ${catkin_LIBRARIES} ncurses)
# target_link_libraries(keyboard_control ${ncurses++_LIBRARIES})
# add_dependencies(keyboard_control ${catkin_EXPORTED_TARGETS} keyboard_control)


#include_directories(${OCTOMAP_INCLUDE_DIRS})
#link_libraries(${OCTOMAP_LIBRARIES})


#include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )
#add_executable(ground_plane src/ground_plane.cpp)
#add_definitions(${PCL_DEFINITIONS})
#target_link_libraries(ground_plane ${catkin_LIBRARIES} ${PCL_LIBRARY_DIRS})
#add_dependencies(ground_plane ${catkin_EXPORTED_TARGETS} ground_plane)


# include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )
add_executable(floor_finder src/floor_finder.cpp)
# add_definitions(${PCL_DEFINITIONS})
target_link_libraries(floor_finder ${catkin_LIBRARIES} ${PCL_LIBRARY_DIRS})
add_dependencies(floor_finder ${catkin_EXPORTED_TARGETS} floor_finder)


add_executable(ground_plane_cheat src/ground_plane_cheat.cpp)
target_link_libraries(ground_plane_cheat ${catkin_LIBRARIES})
add_dependencies(ground_plane_cheat ${catkin_EXPORTED_TARGETS} ground_plane_cheat)

#include_directories(${GAZEBO_INCLUDE_DIRS})
#link_directories(${GAZEBO_LIBRARY_DIRS})
#add_executable(respawn_robot src/respawn_robot.cpp)
#target_link_libraries(respawn_robot ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
#add_dependencies(respawn_robot ${catkin_EXPORTED_TARGETS} respawn_robot)

add_executable(conical_filter src/conical_filter.cpp)
target_link_libraries(conical_filter ${catkin_LIBRARIES})
add_dependencies(conical_filter ${catkin_EXPORTED_TARGETS} conical_filter)

# DON'T NEED TO ADD THIS AS AN EXECUTABLE
# SINCE IT'S ALREADY ADDED AS A LIBRARY...
# add_executable(custom_planner src/custom_planner.cpp)
# target_link_libraries(custom_planner ${catkin_LIBRARIES})


## This plugin includes Qt widgets, so we must include Qt like so:
# include(${QT_USE_FILE})
