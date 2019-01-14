cmake_minimum_required(VERSION 2.8.3)
project(slocum_backseat)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/serial_backseat_interface
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
