cmake_minimum_required(VERSION 2.8.3)
project(slocum_glider_pwrctl)

find_package(
  catkin REQUIRED
  COMPONENTS
  roslint
)

catkin_package()


#catkin_python_setup()
# Not needed because we don't have a src/package python package, only a script.

file(GLOB_RECURSE ${PROJECT_NAME}_PYTHON_SRCS *.py)
roslint_python(${${PROJECT_NAME}_PYTHON_SRCS} scripts/slocum_glider_pwrctl)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/slocum_glider_pwrctl
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/slocum_glider_pwrctl.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
