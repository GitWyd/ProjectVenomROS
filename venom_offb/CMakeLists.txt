cmake_minimum_required(VERSION 2.8.3)
project(venom_offb)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
	roscpp
	std_msgs
	geometry_msgs
	nav_msgs
	mavros_msgs
	eigen_conversions
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS eigen_conversions
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${Eigen_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  include/Navigator.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(offb_circle src/offb_circle.cpp)
target_link_libraries(offb_circle ${catkin_LIBRARIES})

add_executable(fake_ai_node src/fake_ai_node.cpp)
target_link_libraries(fake_ai_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(navigation_node src/navigation_node.cpp)
target_link_libraries(navigation_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(vision_pose_converter src/vision_pose_converter.cpp)
target_link_libraries(vision_pose_converter ${catkin_LIBRARIES})

add_executable(circle src/circle.cpp)
target_link_libraries(circle ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(hunt_drone src/hunt_drone.cpp)
target_link_libraries(hunt_drone ${catkin_LIBRARIES} ${PROJECT_NAME})
