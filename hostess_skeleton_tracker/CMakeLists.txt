cmake_minimum_required(VERSION 2.8.3)

project(hostess_skeleton_tracker)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	std_msgs
	roscpp
	roslib
	tf
	cob_perception_msgs
	cyton_wrapper
	pluginlib
)

find_package(OpenCV REQUIRED)

# Find OpenNI
find_package(PkgConfig)
pkg_check_modules(OpenNI REQUIRED libopenni)

catkin_package(
	INCLUDE_DIRS include
)

include_directories(
	include
	${catkin_INCLUDEDIR}
    ${OpenNI_INCLUDEDIR}
    ${orocos_kdl_INCLUDE_DIRS}
    ${cyton_wrapper_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_executable(hostess_skeleton_kalman src/hostess_skeleton_kalman.cpp)

add_dependencies(hostess_skeleton_kalman geometry_msgs_gencpp)

target_link_libraries(hostess_skeleton_kalman
	${catkin_LIBRARIES}
	${OpenNI_LIBRARIES}
	${orocos_kdl_LIBRARIES}
	${OpenCV_LIBS}
	cv_bridge
	image_transport
)

add_library(pan_controller
	src/pan_controller.cpp
)

add_executable(dynamixel_mediator src/dynamixel_mediator.cpp)
target_link_libraries(dynamixel_mediator pan_controller ${catkin_LIBRARIES})

add_executable(logger src/logger.cpp)
target_link_libraries(logger ${catkin_LIBRARIES})