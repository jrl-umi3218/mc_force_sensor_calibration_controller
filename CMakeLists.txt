cmake_minimum_required(VERSION 3.1)

# Required for Ceres
set(CMAKE_CXX_STANDARD 14)

set(PROJECT_NAME ForceSensorCalibration)
set(PROJECT_DESCRIPTION "ForceSensorCalibration")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

find_package(mc_rtc REQUIRED)
find_package(Ceres REQUIRED)

add_subdirectory(src)

configure_file(etc/ForceSensorCalibration.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/ForceSensorCalibration.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/ForceSensorCalibration.yaml" DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/etc")
