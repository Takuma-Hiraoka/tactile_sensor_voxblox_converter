cmake_minimum_required(VERSION 3.0.2)
project(tactile_sensor_voxblox_converter)

set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  voxblox
  voxblox_ros
  voxblox_msgs
  tf2_ros
  )
find_package(PkgConfig REQUIRED)
find_package(choreonoid REQUIRED)

catkin_package(
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
 )

add_executable(tactile_sensor_voxblox_converter src/tactile_sensor_voxblox_converter.cpp)
target_link_libraries(tactile_sensor_voxblox_converter ${catkin_LIBRARIES} ${CHOREONOID_BODY_LIBRARIES})
