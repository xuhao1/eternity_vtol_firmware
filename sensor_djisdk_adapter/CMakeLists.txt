cmake_minimum_required(VERSION 2.8.3)
project(sensor_djisdk_adapter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()
# catkin_python_setup()
 install(PROGRAMS
    scripts/unreal_sensor_adapter.py
	scripts/sensor_to_unreal.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/demo.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
