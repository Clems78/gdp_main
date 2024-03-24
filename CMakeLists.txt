cmake_minimum_required(VERSION 3.0.2)
project(gdp_main)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros
  std_msgs
  geometry_msgs
  mavros_msgs
  rospy
  sensor_msgs
  iq_gnc
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
 CoordinateList.msg
 DronePositionList.msg
#   Message1.msg
#   Message2.msg
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
  geometry_msgs
#   std_msgs  # Or other packages containing msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gdp_main
   CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../iq_gnc/include
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/gdp_main.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/gdp_main_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gdp_main.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(multi_square src/multi_square.cpp)
target_link_libraries(multi_square ${catkin_LIBRARIES})


add_executable(sub src/sub.cpp)
target_link_libraries(sub ${catkin_LIBRARIES})

add_executable(search_detect_land_g src/search_detect_land_g.cpp)
target_link_libraries(search_detect_land_g ${catkin_LIBRARIES})

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES})

add_executable(lla_test src/lla_test.cpp)
target_link_libraries(lla_test ${catkin_LIBRARIES})

#add_executable(tracked_vehicles src/tracked_vehicles.cpp)
#target_link_libraries(tracked_vehicles ${catkin_LIBRARIES})

add_executable(square src/square.cpp)
target_link_libraries(square ${catkin_LIBRARIES})

add_executable(uselocation src/uselocation.cpp)
target_link_libraries(uselocation ${catkin_LIBRARIES})

add_executable(waypoints_global src/waypoints_global.cpp)
target_link_libraries(waypoints_global ${catkin_LIBRARIES})

add_executable(search_global src/search_global.cpp)
target_link_libraries(search_global ${catkin_LIBRARIES})

add_executable(search_global2 src/search_global2.cpp)
target_link_libraries(search_global2 ${catkin_LIBRARIES})

add_executable(swarm_position src/swarm_position.cpp)
target_link_libraries(swarm_position ${catkin_LIBRARIES})

add_executable(apf_test_drone_1 src/apf_test_drone_1.cpp)
target_link_libraries(apf_test_drone_1 ${catkin_LIBRARIES})

add_executable(apf_global_drone_1 src/apf_global_drone_1.cpp)
target_link_libraries(apf_global_drone_1 ${catkin_LIBRARIES})

add_executable(apf_global_drone_2 src/apf_global_drone_2.cpp)
target_link_libraries(apf_global_drone_2 ${catkin_LIBRARIES})

add_executable(apf1 src/apf1.cpp)
target_link_libraries(apf1 ${catkin_LIBRARIES})

add_executable(apf2 src/apf2.cpp)
target_link_libraries(apf2 ${catkin_LIBRARIES})

add_executable(uselocation1 src/uselocation1.cpp)
target_link_libraries(uselocation1 ${catkin_LIBRARIES})

add_executable(uselocation2 src/uselocation2.cpp)
target_link_libraries(uselocation2 ${catkin_LIBRARIES})


add_executable(search_detect_store_drone2 src/search_detect_store_drone2.cpp)
target_link_libraries(search_detect_store_drone2 ${catkin_LIBRARIES})



################
## REAL FLIGHT##
################

add_executable(waypoints src/Flight_Arena_Test/waypoints.cpp)
target_link_libraries(waypoints ${catkin_LIBRARIES})

add_executable(search_detect_land_fl src/Flight_Arena_Test/search_detect_land_fl.cpp)
target_link_libraries(search_detect_land_fl ${catkin_LIBRARIES})

add_executable(global_waypoints_fl src/Flight_Arena_Test/global_waypoints_fl.cpp)
target_link_libraries(global_waypoints_fl ${catkin_LIBRARIES})

add_executable(tracking_test src/Flight_Arena_Test/tracking_test.cpp)
target_link_libraries(tracking_test ${catkin_LIBRARIES})

add_executable(tracking_gpt src/Flight_Arena_Test/tracking_gpt.cpp)
target_link_libraries(tracking_gpt ${catkin_LIBRARIES})

add_executable(tracking src/Flight_Arena_Test/tracking.cpp)
target_link_libraries(tracking ${catkin_LIBRARIES})

add_executable(tracking_yolov5 src/Flight_Arena_Test/tracking_yolov5.cpp)
target_link_libraries(tracking_yolov5 ${catkin_LIBRARIES})

add_executable(tarkyollo src/Flight_Arena_Test/tarkyollo.cpp)
target_link_libraries(tarkyollo ${catkin_LIBRARIES})



add_executable(tk2 src/Flight_Arena_Test/tk2.cpp)
target_link_libraries(tk2 ${catkin_LIBRARIES})

add_executable(tk3 src/Flight_Arena_Test/tk3.cpp)
target_link_libraries(tk3 ${catkin_LIBRARIES})

add_executable(search_detect_drone2 src/Flight_Arena_Test/search_detect_drone2.cpp)
target_link_libraries(search_detect_drone2 ${catkin_LIBRARIES})

add_executable(search_detect_hv_sl src/Flight_Arena_Test/search_detect_hv_sl.cpp)
target_link_libraries(search_detect_hv_sl ${catkin_LIBRARIES})