cmake_minimum_required(VERSION 3.0.2)
project(kd_noise_publisher)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(kd_noise_publisher_node kd_noise_publisher_node.cpp)

target_link_libraries(kd_noise_publisher_node
  ${catkin_LIBRARIES}
)

install(TARGETS kd_noise_publisher_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
