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
  dynamic_reconfigure
)

find_package( PCL REQUIRED )

generate_dynamic_reconfigure_options(
  cfg/fusion.cfg
)

set( EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR} )

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs tf tf_conversions pcl_ros dynamic_reconfigure
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
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)


add_executable(pointcloud_fusion src/pointcloudFusion.cpp)
target_link_libraries(pointcloud_fusion ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pointcloud_fusion ${PROJECT_NAME}_gencfg)

add_executable(pointcloud_icp src/pointcloudIcp.cpp)
target_link_libraries(pointcloud_icp ${catkin_LIBRARIES} ${PCL_LIBRARIES})
