cmake_minimum_required(VERSION 2.8.3)
project(slocum_glider_extctl_serial)

find_package(
  catkin REQUIRED
  COMPONENTS
  roslint
)

catkin_package()

catkin_python_setup()

file(GLOB_RECURSE ${PROJECT_NAME}_PYTHON_SRCS *.py)
roslint_python(${${PROJECT_NAME}_PYTHON_SRCS} scripts/slocum_glider_extctl_serial)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/slocum_glider_extctl_serial
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
