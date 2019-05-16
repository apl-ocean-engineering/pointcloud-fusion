cmake_minimum_required(VERSION 2.8.3)
project(sensor_fusion)

add_compile_options(-std=c++14)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
)

find_package( PCL REQUIRED )

set( EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR} )

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs tf tf_conversions pcl_ros
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(pointcloud_fusion src/pointcloudFusion.cpp)
target_link_libraries(pointcloud_fusion ${catkin_LIBRARIES} ${PCL_LIBRARIES})
