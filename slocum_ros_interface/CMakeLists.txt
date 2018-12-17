cmake_minimum_required(VERSION 2.8.3)
project(slocum_ros_interface)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/serial_interface
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
