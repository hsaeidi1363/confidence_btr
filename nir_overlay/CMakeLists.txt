cmake_minimum_required(VERSION 2.8.3)
project(nir_overlay)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  tf
  tf_conversions
  )

find_package(OpenCV REQUIRED)

#rsd - this is required because PCL cmake files are here
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/")

find_package( 
  #rsd - this line needs to be separate to get 1.7 
  PCL 1.7 REQUIRED COMPONENTS PATHS /usr/lib/x86_64-linux-gnu/ NO_DEFAULT_PATH
  )

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES  
  CATKIN_DEPENDS
  DEPENDS #PCL #1.7
  )   

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  #/usr/lib/x86_64-linux-gnu/
  )

link_directories(${PCL_LIBRARY_DIRS})
link_directories( ${catkin_LIBRARIES} ${PCL_LIBRARIES} )
add_definitions(${PCL_DEFINITIONS})



add_executable(nir_overlay_intel src/node_intel.cpp )
target_link_libraries(nir_overlay_intel ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(overlay_2D src/overlay_2D.cpp )
target_link_libraries(overlay_2D ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})


#add_executable(intel_pcl_filter src/intel_pcl_filter.cpp )
#target_link_libraries(intel_pcl_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
