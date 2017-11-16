cmake_minimum_required(VERSION 2.8.3)
project(venom_zed)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  roscpp)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
find_package(ZED 2 REQUIRED)
find_package(CUDA REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${ZED_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES venom_zed
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable( example src/example.cpp)
target_link_libraries( example ${catkin_LIBRARIES} ${ZED_LIBRARIES} ${OpenCV_LIBS})

add_executable( depth_image src/depth_image.cpp)
target_link_libraries( depth_image ${catkin_LIBRARIES} ${ZED_LIBRARIES} ${OpenCV_LIBS})
