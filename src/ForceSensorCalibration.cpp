#include "ForceSensorCalibration.h"

ForceSensorCalibration::ForceSensorCalibration(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  if(config.has(rm->name))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No configuration section for {}, add a configuration for this robot before attempting calibration", rm->name);
  }
  mc_rtc::log::success("ForceSensorCalibration init done");
}

bool ForceSensorCalibration::run()
{
  return mc_control::fsm::Controller::run();
}

void ForceSensorCalibration::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}


