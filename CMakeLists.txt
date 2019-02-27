cmake_minimum_required(VERSION 2.8.3)
project(pcl_server)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometric_shapes
  geometry_msgs
  shape_msgs
  std_msgs
  message_generation
  pcl_ros
  roscpp
  rospy
  roslib
  )

find_package(PCL 1.7 REQUIRED COMPONENTS common io)


include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()

add_service_files(
  FILES
  ComputeConvexHull.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  shape_msgs
)

catkin_package(
 #  INCLUDE_DIRS include
   CATKIN_DEPENDS geometric_shapes geometry_msgs shape_msgs roscpp roslib visualization_msgs message_runtime
#  DEPENDS system_lib
)
## Generate services in the 'srv' folder




include_directories(  
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_executable(pcl_server_node ./src/pcl_server.cpp )
add_dependencies(pcl_server_node pcl_server_generate_messages_cpp)
target_link_libraries(pcl_server_node ${catkin_LIBRARIES})
