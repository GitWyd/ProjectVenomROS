cmake_minimum_required(VERSION 2.8.3)
project(venom_offb)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
	roscpp
	std_msgs
	geometry_msgs
	nav_msgs
	mavros_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES venom_ros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/venom_ros.cpp
# )

## Declare a C++ executable
add_executable(offb_node src/offb_node.cpp)
target_link_libraries(offb_node ${catkin_LIBRARIES})

add_executable(offb_circle src/offb_circle.cpp)
target_link_libraries(offb_circle ${catkin_LIBRARIES})
