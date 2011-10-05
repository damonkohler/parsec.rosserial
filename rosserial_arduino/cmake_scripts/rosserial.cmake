cmake_minimum_required(VERSION 2.6)

#lets set the name of the project to the name of the pkg
#this emulates the typical rosbuild
get_filename_component(_project ${CMAKE_SOURCE_DIR} NAME)
project(${_project})

set(CMAKE_MODULE_PATH ${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/modules)  # CMake module search path
include(${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/toolchains/Arduino.cmake) # Arduino Toolchain

find_package(Arduino)

rosbuild_find_ros_package(rosserial_client)
FILE(GLOB_RECURSE ROS_CLIENT_SRCS ${rosserial_client_PACKAGE_PATH}/src/ros_lib/*)
include_directories(${rosserial_client_PACKAGE_PATH}/src/ros_lib)
FILE(GLOB_RECURSE ROS_ARDUINO_SRCS ${rosserial_arduino_PACKAGE_PATH}/src/ros_lib/*)
include_directories(${rosserial_arduino_PACKAGE_PATH}/src/ros_lib)

#get a list of all the pkgs in the manifest
#so that we can generate the rosserial implementation for them
rosbuild_invoke_rospack(${PROJECT_NAME} ${PROJECT_NAME} "depedencies" "depends")
string(REGEX REPLACE "\n" ";" ${PROJECT_NAME}_depedencies ${${PROJECT_NAME}_depedencies})

foreach(MSG_PKG ${${PROJECT_NAME}_depedencies})
        rosbuild_find_ros_package(${MSG_PKG})
	if (EXISTS ${${MSG_PKG}_PACKAGE_PATH}/msg AND
		NOT EXISTS ${PROJECT_SOURCE_DIR}/msg_gen/${MSG_PKG})
		message(STATUS "Generating rosserial implementation for ${MSG_PKG} in ${PROJECT_SOURCE_DIR}/src" )
		execute_process(COMMAND rosrun rosserial_client make_library.py ${PROJECT_SOURCE_DIR}/msg_gen ${MSG_PKG} OUTPUT_QUIET)
	endif()
endforeach(MSG_PKG)
FILE(GLOB_RECURSE ROS_MSG_GEN ${PROJECT_SOURCE_DIR}/msg_gen/*)
include_directories(${PROJECT_SOURCE_DIR}/msg_gen)

# - Generate firmware for rosserial_arduino Devices
# generate_ros_firmware(TARGET_NAME)
#        TARGET_NAME - Name of target
# Creates a Arduino firmware target.
#
# The target options can be configured by setting options of
# the following format:
#      ${TARGET_NAME}${SUFFIX}
# The following suffixes are availabe:
#      _SRCS           # Sources
#      _HDRS           # Headers
#      _LIBS           # Libraries to linked in
#      _BOARD          # Board name (such as uno, mega2560, ...)
#      _PORT           # Serial port, for upload and serial targets [OPTIONAL]
#      _AFLAGS         # Override global Avrdude flags for target
#      _SERIAL         # Serial command for serial target           [OPTIONAL]
#      _NO_AUTOLIBS    # Disables Arduino library detection
#      _NO_DEFAULT_COM # Disables a default communication implementation
#
# Here is a short example for a target named test:
#       set(test_SRCS  test.cpp)
#       set(test_HDRS  test.h)
#       set(test_BOARD uno)
#       generate_ros_firmware(test)

macro(generate_ros_firmware TARGET_NAME)
	set(${TARGET_NAME}_SRCS
            ${${TARGET_NAME}_SRCS}
            ${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/cc_support.cpp
            ${ROS_CLIENT_SRCS}
            ${ROS_ARDUINO_SRCS}
            ${ROS_MSG_GEN})
	generate_arduino_firmware(${TARGET_NAME})
endmacro()
