cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_find_ros_package(rosserial_arduino)

set(ARDUINO_SDK_PATH /usr/share/arduino-0022)
include(${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/rosserial.cmake)

set(FIRMWARE_NAME time_test)
set(${FIRMWARE_NAME}_BOARD mega2560)  # Arduino Target board
file(GLOB ${FIRMWARE_NAME}_HDRS src/time_test/*.h)
file(GLOB ${FIRMWARE_NAME}_SRCS src/time_test/*.cpp)
set(${FIRMWARE_NAME}_PORT /dev/ttyUSB0)  # Serial upload port

generate_ros_firmware(${FIRMWARE_NAME})

set(FIRMWARE_NAME int32_test)
set(${FIRMWARE_NAME}_BOARD mega2560)  # Arduino Target board
file(GLOB ${FIRMWARE_NAME}_HDRS src/int32_test/*.h)
file(GLOB ${FIRMWARE_NAME}_SRCS src/int32_test/*.cpp)
set(${FIRMWARE_NAME}_PORT /dev/ttyUSB0)  # Serial upload port

generate_ros_firmware(${FIRMWARE_NAME})

set(FIRMWARE_NAME float64_test)
set(${FIRMWARE_NAME}_BOARD mega2560)  # Arduino Target board
file(GLOB ${FIRMWARE_NAME}_HDRS src/float64_test/*.h)
file(GLOB ${FIRMWARE_NAME}_SRCS src/float64_test/*.cpp)
set(${FIRMWARE_NAME}_PORT /dev/ttyUSB0)  # Serial upload port

generate_ros_firmware(${FIRMWARE_NAME})

set(FIRMWARE_NAME array_test)
set(${FIRMWARE_NAME}_BOARD mega2560)  # Arduino Target board
file(GLOB ${FIRMWARE_NAME}_HDRS src/array_test/*.h)
file(GLOB ${FIRMWARE_NAME}_SRCS src/array_test/*.cpp)
set(${FIRMWARE_NAME}_PORT /dev/ttyUSB0)  # Serial upload port

generate_ros_firmware(${FIRMWARE_NAME})
