cmake_minimum_required(VERSION 3.0.2)
project(imu_turn180)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_turn180_node src/imu_turn180.cpp)
target_link_libraries(imu_turn180_node ${catkin_LIBRARIES})

install(TARGETS imu_turn180_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
