cmake_minimum_required(VERSION 3.0.2)
project(obstacle_avoid)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(obstacle_avoid_node src/obstacle_avoid_node.cpp)
target_compile_features(obstacle_avoid_node PRIVATE cxx_std_14)
target_link_libraries(obstacle_avoid_node ${catkin_LIBRARIES})

install(TARGETS obstacle_avoid_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
