cmake_minimum_required(VERSION 2.8.3)
project(venom_perception)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
	roscpp
	std_msgs
	sensor_msgs
	geometry_msgs
	nav_msgs
	mavros_msgs
	pcl_ros
	pcl_conversions
	image_transport
	cv_bridge
	tf
)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES venom_ros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/venom_ros.cpp
# )

set(link_directories ${OpenCV_LIBS} ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(depth_map src/depth_map.cpp)
target_link_libraries(depth_map ${link_directories})

add_executable(example src/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES})

add_executable(save_zed src/save_zed.cpp)
target_link_libraries(save_zed ${catkin_LIBRARIES})
