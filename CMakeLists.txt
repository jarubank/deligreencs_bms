cmake_minimum_required(VERSION 3.0.2)
project(deligreencs_bms)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_install_python(PROGRAMS
  scripts/deligreencs_bms
  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
