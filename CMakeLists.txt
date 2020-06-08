cmake_minimum_required(VERSION 2.8)
project(advanced_navi)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  move_base_msgs
  std_srvs
  sensor_msgs
  gazebo_msgs
)

catkin_package(
CATKIN_DEPENDS message_runtime
)

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

# TODO 进行实验 2.2 时取消注释
add_executable(init_pose ${CMAKE_CURRENT_LIST_DIR}/src/InitPose/InitPose.cc)
target_link_libraries(init_pose ${catkin_LIBRARIES})

# TODO 进行实验 2.3 时取消注释
add_executable(set_goal ${CMAKE_CURRENT_LIST_DIR}/src/SetGoal/SetGoal.cc)
target_link_libraries(set_goal ${catkin_LIBRARIES})

# TODO 进行实验 2.4 时取消注释
# add_executable(sequence_goal ${CMAKE_CURRENT_LIST_DIR}/src/SeqGoal/SeqGoal.cc)
# target_link_libraries(sequence_goal ${catkin_LIBRARIES})
