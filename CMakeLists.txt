cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME ForceSensorCalibration)
set(PROJECT_DESCRIPTION "ForceSensorCalibration")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

find_package(mc_rtc REQUIRED)

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/ForceSensorCalibration.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/ForceSensorCalibration.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/ForceSensorCalibration.yaml" DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/etc")

# Install calibration script
set(CALIBRATION_SCRIPT_PATH "${MC_CONTROLLER_INSTALL_PREFIX}/${PROJECT_NAME}/scripts")
set(CALIBRATION_SCRIPT_EXECUTABLE "${CALIBRATION_SCRIPT_PATH}/calib_force.py")
install(PROGRAMS force_sensors_calibration/calib_force.py DESTINATION "${CALIBRATION_SCRIPT_PATH}")
configure_file(src/config.h.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/src/config.h)

