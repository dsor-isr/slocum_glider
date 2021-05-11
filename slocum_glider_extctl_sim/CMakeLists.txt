cmake_minimum_required(VERSION 2.8.3)
project(slocum_glider_extctl_sim)

find_package(
  catkin REQUIRED
  COMPONENTS
  roslint
)

catkin_package()

catkin_python_setup()

set(ROSLINT_PYTHON_OPTS "--ignore=W503,E241,E226,E123")
file(GLOB_RECURSE ${PROJECT_NAME}_PYTHON_SRCS *.py)
roslint_python(${${PROJECT_NAME}_PYTHON_SRCS} scripts/slocum_glider_extctl_sim scripts/slocum_glider_sim_console)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/slocum_glider_extctl_sim
  scripts/slocum_glider_sim_console
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install the example Glider mission and config files.
install(DIRECTORY example_gliders
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  launch/sim_driver.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
