cmake_minimum_required(VERSION 2.8.3)
project(venom_perception)

add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED
  OpenCV
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
 ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(depth_map src/depth_map.cpp)
target_link_libraries(depth_map ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(example src/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES})

add_executable(save_zed src/save_zed.cpp)
target_link_libraries(save_zed ${catkin_LIBRARIES})

add_executable(local_map src/local_map.cpp)
target_link_libraries(local_map ${catkin_LIBRARIES})

add_executable(detect_drone src/detect_drone.cpp)
target_link_libraries(detect_drone ${catkin_LIBRARIES} ${OpenCV_LIBS})
