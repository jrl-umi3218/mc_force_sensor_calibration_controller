cmake_minimum_required(VERSION 3.1)

# Required for Ceres
set(CMAKE_CXX_STANDARD 14)

set(PROJECT_NAME ForceSensorCalibration)
set(PROJECT_DESCRIPTION "ForceSensorCalibration")
set(PROJECT_URL "")

project(${PROJECT_NAME} VERSION 1.0.0 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting default build type to RelWithDebInfo as none was provided")
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of a build" FORCE)
endif()


find_package(mc_rtc REQUIRED)
find_package(Ceres REQUIRED)

add_subdirectory(src)

configure_file(etc/ForceSensorCalibration.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/ForceSensorCalibration.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/ForceSensorCalibration.yaml" DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/etc")
