cmake_minimum_required(VERSION 3.0.2)
project(imu_exo_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_exo_control
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

## Add ROS dependencies
if(USE_ROS)
	#ROS local compile: use catkin
	if(NOT CMAKE_CROSSCOMPILING)
		message("--catkin--")
		# Required ROS packages
		find_package(
				catkin REQUIRED COMPONENTS
				roscpp
				rospy
				std_msgs
				std_srvs
				sensor_msgs
				geometry_msgs
				visualization_msgs
				dynamic_reconfigure
				message_generation
				rosserial_python
				ros_tcp_endpoint
				cob_srvs
				trigno_msgs
        )
		find_package(OsqpEigen REQUIRED)
		if(SIM)
			find_package(catkin REQUIRED COMPONENTS
			controller_manager_msgs
			cob_gazebo_ros_control
			x2_description
			)
		endif()

		generate_dynamic_reconfigure_options(
                config/EMGReader.cfg
		)

		add_message_files(
				FILES
				X2Array.msg
				X2Acceleration.msg
				X2AccelerationMerge.msg
				X2PressurePad.msg
				X2StanceInterpolation.msg
				X2RobotState.msg
				InteractionArray.msg
		)

		generate_messages(
				DEPENDENCIES
				std_msgs
				sensor_msgs
				geometry_msgs
				trigno_msgs
		)

		catkin_package(
				#  INCLUDE_DIRS include
				#  LIBRARIES x2
			  CATKIN_DEPENDS
				roscpp
				rospy
				std_msgs
				std_srvs
				sensor_msgs
				geometry_msgs
				visualization_msgs
				dynamic_reconfigure
				message_runtime
				rosserial_python
				ros_tcp_endpoint
				trigno_msgs
				#  DEPENDS system_lib
		)

		#include CATKIN
		include_directories(${catkin_INCLUDE_DIRS})
		set(ROS_LIBRARIES ${catkin_LIBRARIES})
	#Cross-compile with ROS (w/o catkin)
	else()
		# Required ROS packages
		find_package(roscpp REQUIRED CONFIG ONLY_CMAKE_FIND_ROOT_PATH)
		find_package(std_msgs REQUIRED CONFIG ONLY_CMAKE_FIND_ROOT_PATH)
		find_package(trigno_msgs REQUIRED CONFIG ONLY_CMAKE_FIND_ROOT_PATH)
		find_package(sensor_msgs REQUIRED CONFIG ONLY_CMAKE_FIND_ROOT_PATH)
		find_package(geometry_msgs REQUIRED CONFIG ONLY_CMAKE_FIND_ROOT_PATH)
		# If cross-compiling force search in sysroot
		if(IS_DIRECTORY ${CMAKE_SYSROOT})
			message("-- Using sysroot folder: ${CMAKE_SYSROOT}")
			#Reset include dirs towards sysroot only
			foreach(inc_dir ${roscpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
				string(REGEX REPLACE "^/usr/" "${CMAKE_SYSROOT}usr/" inc_dir_cross ${inc_dir})
				include_directories(${inc_dir_cross})
			endforeach()
			#Reset libraries towards sysroot only
			foreach(lib_path ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES})
				string(REPLACE "/" ";" lib_path_list ${lib_path}) #Breakdown path in list
				list(REVERSE lib_path_list)
				list(GET lib_path_list 0 lib) #Get last element (library name)
				find_library(new_path ${lib} PATHS ${CMAKE_SYSROOT}) #Force to look for it in actual sysroot path
				list(APPEND ROS_LIBRARIES ${new_path}) #Add it
				unset(new_path CACHE) #Clear variable to allow new search
			endforeach()
		else()
			message(FATAL_ERROR "No sysroot (CMAKE_SYSROOT) defined or not an accessible path")
		endif()
	endif()
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_imu_exo_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
