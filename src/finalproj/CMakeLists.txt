cmake_minimum_required(VERSION 2.8.3)
project(finalproj)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  tf
  tf2
  tf2_msgs
  std_msgs
  sensor_msgs
  laser_geometry
  actionlib
  tf2_ros
  tf2_geometry_msgs
  message_filters
  visualization_msgs
  pcl_conversions
  pcl_ros

)


find_package(PCL REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})



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
#  LIBRARIES assignment
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}



)
add_executable(exploreNode src/exploreNode.cpp)
add_executable(obRec src/obRec.cpp)
add_executable(testNode src/testNode.cpp)


target_link_libraries(exploreNode ${catkin_LIBRARIES})
target_link_libraries(obRec ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(testNode ${catkin_LIBRARIES} ${PCL_LIBRARIES})





