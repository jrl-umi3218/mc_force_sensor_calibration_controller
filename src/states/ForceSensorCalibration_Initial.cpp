#include "ForceSensorCalibration_Initial.h"

#include "../ForceSensorCalibration.h"

void ForceSensorCalibration_Initial::configure(const mc_rtc::Configuration & config)
{
}

void ForceSensorCalibration_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ForceSensorCalibration &>(ctl_);
}

bool ForceSensorCalibration_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ForceSensorCalibration &>(ctl_);
  output("OK");
  return true;
}

void ForceSensorCalibration_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ForceSensorCalibration &>(ctl_);
}

EXPORT_SINGLE_STATE("ForceSensorCalibration_Initial", ForceSensorCalibration_Initial)
