cmake_minimum_required(VERSION 2.8.3)
project(venom_zed)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  roscpp)
find_package(CUDA REQUIRED)
find_package(ZED 2.1)
find_package(OpenCV 3 COMPONENTS core highgui imgproc)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  #${PCL_INCLUDE_DIRS}
)

link_directories(
  ${ZED_LIBRARY_DIR}
  ${CUDA_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  #${PCL_LIBRARY_DIRS}
)

if(NOT DEFINED CUDA_NPP_LIBRARIES_ZED)
    #To deal with cuda 9 nppi libs and previous versions of ZED SDK
    set(CUDA_NPP_LIBRARIES_ZED ${CUDA_npp_LIBRARY} ${CUDA_npps_LIBRARY} ${CUDA_nppi_LIBRARY})   
endif()


set( CUDA_LINK_LIBRARIES
  ${catkin_LIBRARIES}
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARIES} ${CUDA_NPP_LIBRARIES_ZED}
  ${OpenCV_LIBS}
  #${PCL_LIBRARIES}
)

add_executable( example src/example.cpp)
target_link_libraries( example ${catkin_LIBRARIES} ${ZED_LIBRARIES} ${OpenCV_LIBS} )

add_executable( depth_image src/depth_image.cpp)
target_link_libraries( depth_image ${catkin_LIBRARIES} ${ZED_LIBRARIES} ${OpenCV_LIBS})

add_executable( quaternion_test src/quaternion_test.cpp)
target_link_libraries( quaternion_test ${CUDA_LINK_LIBRARIES})

add_executable( venom_zed_test src/venom_zed_test.cpp)
target_link_libraries( venom_zed_test ${CUDA_LINK_LIBRARIES})
