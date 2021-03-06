cmake_minimum_required(VERSION 2.8.3)
project(tissue_waypoints)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  pcl_ros
  sensor_msgs
  message_generation
  trajectory_msgs
  roslib
  iiwa_msgs
  cv_bridge
)

find_package( orocos_kdl REQUIRED )
find_package(OpenCV REQUIRED)

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

## 11/30 Generate messages in the 'msg' folder
 add_message_files(DIRECTORY msg)

## 11/30 Generate services in the 'srv' folder
 add_service_files(FILES Trajectory.srv)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## 11/30 Generate added messages and services with any dependencies listed here 
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )

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
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mrepathplanning
# 11/30
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs iiwa_msgs
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
  ${PCL_INCLUDE_DIRS}
)

find_package(PCL 1.2 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/mrepathplanning.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/mrepathplanning_node.cpp)

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
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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

add_executable(tissue_waypoints 
     src/tissue_waypoints.cpp
     src/pcl_funcs/findNearestPoint.cpp
     src/pcl_funcs/pclIo.cpp 
     src/pcl_funcs/pclCloudViewer.cpp 
     src/pcl_funcs/pclVoxel.cpp 
     src/pcl_funcs/pclPassThrough.cpp 
     src/pcl_funcs/pclStatisticalOutlierRemoval.cpp 
     src/pcl_funcs/pclMlsSmoothing.cpp
     src/pcl_funcs/pclFastTriangular.cpp
     src/pcl_funcs/s_hull_pro.cpp
     src/pcl_funcs/delaunay3.cpp
     src/pcl_funcs/dijkstraPQ.cpp
     )
target_link_libraries(tissue_waypoints ${catkin_LIBRARIES} ${PCL_LIBRARIES} )
#add_dependencies(tissue_waypoints ${PROJECT_NAME}_gencpp iiwa_msgs_generate_messages_cpp)

add_executable(waypoints_client src/waypoints_client.cpp)
target_link_libraries(waypoints_client ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(waypoints_linear src/waypoints_linear.cpp)
target_link_libraries(waypoints_linear ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(waypoints_tumor src/waypoints_tumor.cpp)
target_link_libraries(waypoints_tumor ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(tissue_planner src/planner.cpp)
target_link_libraries(tissue_planner ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(waypoints_rgbcontour src/waypoints_rgbcontour.cpp)
target_link_libraries(waypoints_rgbcontour ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(tumor_loader src/tumor_loader.cpp )
target_link_libraries(tumor_loader ${catkin_LIBRARIES})
