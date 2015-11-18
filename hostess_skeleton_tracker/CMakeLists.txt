cmake_minimum_required(VERSION 2.8.3)

project(hostess_skeleton_tracker)

find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS geometry_msgs
					roscpp
					roslib
					tf)
# Find OpenNI
find_package(PkgConfig)
pkg_check_modules(OpenNI REQUIRED libopenni)

catkin_package()

include_directories(${catkin_INCLUDEDIR}
		    ${OpenNI_INCLUDEDIR}
		    ${orocos_kdl_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_executable(hostess_skeleton_tracker src/openni_tracker.cpp)

add_dependencies(hostess_skeleton_tracker geometry_msgs_gencpp)

target_link_libraries(hostess_skeleton_tracker ${catkin_LIBRARIES}
				     ${OpenNI_LIBRARIES}
				     ${orocos_kdl_LIBRARIES})

install(TARGETS hostess_skeleton_tracker RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES openni_tracker.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(twist_mediator src/twist_mediator.cpp)
target_link_libraries(twist_mediator ${catkin_LIBRARIES})
