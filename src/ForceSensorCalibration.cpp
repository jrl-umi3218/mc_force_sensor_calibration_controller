#include "ForceSensorCalibration.h"

ForceSensorCalibration::ForceSensorCalibration(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{

  LOG_SUCCESS("ForceSensorCalibration init done " << this)
}

bool ForceSensorCalibration::run()
{
  return mc_control::fsm::Controller::run();
}

void ForceSensorCalibration::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}


