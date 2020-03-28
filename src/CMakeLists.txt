set(controller_SRC
  ForceSensorCalibration.cpp
)

set(controller_HDR
  ForceSensorCalibration.h
)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DForceSensorCalibration_EXPORTS")
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_fsm)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
  RUNTIME DESTINATION bin)

add_controller(${PROJECT_NAME}_controller lib.cpp "")
set_target_properties(${PROJECT_NAME}_controller PROPERTIES OUTPUT_NAME "${PROJECT_NAME}")
set_target_properties(${PROJECT_NAME}_controller PROPERTIES
  ARCHIVE_OUTPUT_DIRECTORY lib/$<CONFIGURATION>
  LIBRARY_OUTPUT_DIRECTORY lib/$<CONFIGURATION>
  RUNTIME_OUTPUT_DIRECTORY lib/$<CONFIGURATION>)
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})

add_subdirectory(states)