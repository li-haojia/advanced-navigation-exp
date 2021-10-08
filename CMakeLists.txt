cmake_minimum_required(VERSION 2.8)
project(octo_mapping)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  move_base_msgs
  std_srvs
  sensor_msgs
  gazebo_msgs
  cv_bridge
  image_transport
  pcl_ros
  tf
  octomap_msgs
  octomap_ros
)

catkin_package(
CATKIN_DEPENDS message_runtime
)

find_package(OpenCV REQUIRED MODULE)

find_package(PCL REQUIRED QUIET)

find_package(octomap REQUIRED )
# 默认在ROS的搜索目录中
# message(STATUS "==> octomap include dir: " ${octomap_INCLUDE_DIRS})
# message(STATUS "==> octomap lib dir: " ${octomap_INCLUDE_DIRS})


include_directories(
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INLCUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

## Add link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
)

# add_executable(image_base ${CMAKE_CURRENT_LIST_DIR}/src/ImageBase/ImageBase.cc)
# target_link_libraries(image_base ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable(point_cloud_map ${CMAKE_CURRENT_LIST_DIR}/src/PointsCloudMap/PointsCloudMap.cc)
target_link_libraries(point_cloud_map ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(octo_map ${CMAKE_CURRENT_LIST_DIR}/src/OctoMap/OctoMap.cc)
target_link_libraries(octo_map ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} octomap)
