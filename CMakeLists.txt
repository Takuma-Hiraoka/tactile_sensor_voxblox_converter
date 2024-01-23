cmake_minimum_required(VERSION 3.0.2)
project(tactile_sensor_voxblox_converter)

set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  )
find_package(PkgConfig REQUIRED)
find_package(choreonoid REQUIRED)

# 相対パスを絶対パスに直す                                                                                                                                                                                                                             
set(CHOREONOID_BODY_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_BODY_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

catkin_package(
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
 )

add_executable(tactile_sensor_voxblox_converter src/tactile_sensor_voxblox_converter.cpp)
target_link_libraries(tactile_sensor_voxblox_converter ${catkin_LIBRARIES} ${CHOREONOID_BODY_LIBRARIES_ABS})
