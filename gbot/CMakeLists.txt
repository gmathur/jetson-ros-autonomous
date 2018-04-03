cmake_minimum_required(VERSION 2.8.3)
project(gbot)

add_definitions(-std=c++11) # Use C++11

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  sensor_msgs
  ydlidar
)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)


