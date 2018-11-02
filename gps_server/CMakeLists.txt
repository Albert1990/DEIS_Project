cmake_minimum_required(VERSION 2.8.3)
project(gps_server)

find_package(catkin REQUIRED 
  roscpp
  cv_bridge
  std_msgs
  message_generation

)
find_package(OpenCV REQUIRED)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp  message_runtime 
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

# There's no version hint in the yaml-cpp headers, so get the version number
# from pkg-config.
find_package(PkgConfig REQUIRED)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")





