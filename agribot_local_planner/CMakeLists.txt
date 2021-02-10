cmake_minimum_required(VERSION 2.8.3)
project(agribot_local_planner)
# c++11
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS 
		roscpp
		tf 
		dynamic_reconfigure 
		geometry_msgs
		cmake_modules 
		costmap_2d
		nav_core 
		nav_msgs 
		pluginlib 
		std_msgs
		geometry_msgs
		base_local_planner
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(
  cfg/AgribotLocalPlanner.cfg
)

catkin_package(
	LIBRARIES agribot_local_planner 
	CATKIN_DEPENDS 
		roscpp
		tf
		dynamic_reconfigure
		costmap_2d
		geometry_msgs
		nav_core
		pluginlib 
)

add_library(agribot_local_planner 
	src/agribot_local_planner.cpp
)

target_link_libraries(
	agribot_local_planner
	${catkin_LIBRARIES}
)

add_dependencies(agribot_local_planner agribot_local_planner_gencfg)

install(TARGETS agribot_local_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES agribot_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
