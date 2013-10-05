cmake_minimum_required(VERSION 2.8.3)
project(rubble)
set(BUILD_FLAGS "-std=c++0x -Wreturn-type -Wno-enum-compare")

# Add additional CMake files
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp roslib
 message_generation genmsg actionlib actionlib_msgs
 std_msgs sensor_msgs geometry_msgs trajectory_msgs diagnostic_msgs visualization_msgs interactive_markers gazebo_msgs
 tf tf_conversions eigen_conversions rviz class_loader urdf)

# Include Eigen for matrices
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

# Include CppUnit
find_package(CppUnit)
include_directories(${CPPUNIT_INCLUDE_DIR})
link_libraries(${CPPUNIT_LIBRARIES})

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
file(GLOB message_source "msg/*.msg")
list(SORT message_source)
foreach(message_src_file ${message_source})
	get_filename_component(message_base ${message_src_file} NAME)
	message(STATUS "Adding message ${message_base}" )
	add_message_files(DIRECTORY msg FILES ${message_base})
endforeach(message_src_file)

## Generate services in the 'srv' folder
file(GLOB service_source "srv/*.srv")
list(SORT service_source)
foreach(service_src_file ${service_source})
	get_filename_component(service_base ${service_src_file} NAME)
	message(STATUS "Adding service ${service_base}" )
	add_service_files(DIRECTORY srv FILES ${service_base})
endforeach(service_src_file)

## Generate actions in the 'action' folder
file(GLOB action_source "action/*.action")
list(SORT action_source)
foreach(action_src_file ${action_source})
	get_filename_component(action_base ${action_src_file} NAME)
	message(STATUS "Adding action ${action_base}" )
	add_action_files(DIRECTORY action FILES ${action_base})
endforeach(action_src_file)

## Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs trajectory_msgs diagnostic_msgs actionlib_msgs gazebo_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
	#LIBRARIES rubble
	CATKIN_DEPENDS roscpp
 message_runtime genmsg actionlib actionlib_msgs
 std_msgs sensor_msgs geometry_msgs trajectory_msgs diagnostic_msgs visualization_msgs interactive_markers gazebo_msgs
 tf tf_conversions eigen_conversions rviz class_loader urdf
	DEPENDS ach rt QtCore QtGui
)

## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})
link_libraries(${catkin_LIBRARIES})

###########
## Build ##
###########

# Collect the source, node and test files
file(GLOB lib_source "src/libraries/*.cpp")
list(SORT lib_source)
file(GLOB nodes_source "src/nodes/*.cpp")
list(SORT nodes_source)
file(GLOB unit_tests_source "test/unit/*.cpp")
list(SORT unit_tests_source)
file(GLOB system_tests_source "test/system/*.cpp")
list(SORT system_tests_source)
file(GLOB include_source "include/rubble/*.h")
list(SORT include_source)

## Declare a cpp library
#add_library(${PROJECT_NAME} SHARED ${lib_source})
#set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS ${BUILD_FLAGS})
## Allow Testing
enable_testing()

# Build unit tests
message(STATUS "\n-- UNIT TESTS: ")
foreach(utest_src_file ${unit_tests_source})
	get_filename_component(test_base ${utest_src_file} NAME_WE)
	message(STATUS "Adding test ${test_base}" )
	add_executable(${test_base} "test/test_units.cpp" ${utest_src_file})
	add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}/${test_base})
	#catkin_add_gtest(${test_base} src/tests/unit/TestParameterizedObject.cpp)
	#target_link_libraries(${test_base} ${PROJECT_NAME})
	set_target_properties(${test_base} PROPERTIES COMPILE_FLAGS ${BUILD_FLAGS})
	add_dependencies(tests ${test_base})
endforeach(utest_src_file)

# Build executables
message(STATUS "\n-- ROS NODES: ")
foreach(node_src_file ${nodes_source})
	get_filename_component(node_base ${node_src_file} NAME_WE)
	message(STATUS "Adding ROS Node ${node_base}" )
		add_executable(${node_base} ${node_src_file} ${include_source})
	#target_link_libraries(${node_base} ${PROJECT_NAME})
	add_dependencies(${node_base} ${PROJECT_NAME}_gencpp)
	set_target_properties(${node_base} PROPERTIES COMPILE_FLAGS ${BUILD_FLAGS})
endforeach(node_src_file)

# Build system tests
message(STATUS "\n-- ROS TESTING NODES: ")
foreach(test_node_src_file ${system_tests_source})
	get_filename_component(test_node_base ${test_node_src_file} NAME_WE)
	message(STATUS "Adding ROS Node ${test_node_base}" )
	add_executable(${test_node_base} ${test_node_src_file})
	#target_link_libraries(${test_node_base} ${PROJECT_NAME})
	add_dependencies(${test_node_base} ${PROJECT_NAME}_gencpp)
	set_target_properties(${test_node_base} PROPERTIES COMPILE_FLAGS ${BUILD_FLAGS})
endforeach(test_node_src_file)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
#install(TARGETS ${PROJECT_NAME}
#	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#	)

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#	FILES_MATCHING PATTERN "*.h"
#	PATTERN ".svn" EXCLUDE
#	)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

