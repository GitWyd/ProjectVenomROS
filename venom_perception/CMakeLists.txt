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

## Declare a C++ executable
add_executable(visual_odometry src/visual_odometry.cpp)
target_link_libraries(visual_odometry ${catkin_LIBRARIES})

add_executable(visual_pose src/visual_pose.cpp)
target_link_libraries(visual_pose ${catkin_LIBRARIES})

add_executable(cloud_reader src/cloud_reader.cpp)
target_link_libraries(cloud_reader ${catkin_LIBRARIES})

add_executable(depth_map src/depth_map.cpp)
target_link_libraries(depth_map ${catkin_LIBRARIES})

add_executable(cv_test src/cv_test.cpp)
target_link_libraries(cv_test ${catkin_LIBRARIES})

add_executable(perception_node src/perception_node.cpp)
target_link_libraries(perception_node ${catkin_LIBRARIES})
