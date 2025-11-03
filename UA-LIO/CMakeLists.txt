cmake_minimum_required(VERSION 3.0.2)
project(ieskf_fusion)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  livox_ros_driver
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  interactive_markers
  tf
)

find_package( PCL REQUIRED )
find_package( Eigen3 REQUIRED )


catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  livox_ros_driver
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(ieskf_fusion
  src/ieskf_fusion_node.cpp
  src/ieskf_fusion_flow.cpp
  src/ieskf/ieskf.cpp
  src/modules/frontend/frontend.cpp
  src/modules/map/map_manager.cpp
  src/modules/map/voxelmap.cpp
  src/modules/propagate/propagate.cpp
  src/modules/subscriber/imu_subscriber.cpp
  src/modules/subscriber/lidar_subscriber.cpp
  src/modules/publisher/tf_broadcaster.cpp
  src/modules/publisher/path_publisher.cpp
  src/modules/publisher/odometry_publisher.cpp
  src/modules/publisher/pointcloud2_publisher.cpp
)

add_dependencies(ieskf_fusion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ieskf_fusion
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(voxelmap_update
  src/voxelmap_update.cpp
  src/modules/map/voxelmap.cpp
)

add_dependencies(voxelmap_update ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(voxelmap_update
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
