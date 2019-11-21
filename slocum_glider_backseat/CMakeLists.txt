cmake_minimum_required(VERSION 2.8.3)
project(slocum_glider_backseat)

find_package(
  catkin REQUIRED
  COMPONENTS
  roslint
)

catkin_package()

catkin_python_setup()

file(GLOB_RECURSE ${PROJECT_NAME}_PYTHON_SRCS *.py)
roslint_python(${${PROJECT_NAME}_PYTHON_SRCS} scripts/serial_backseat_interface)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/serial_backseat_interface
  scripts/bottom_follow
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
