#include "ChooseRobot.h"
#include "../ForceSensorCalibration.h"
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/io_utils.h>
#include <boost/filesystem.hpp>

bool ChooseRobot::run(mc_control::fsm::Controller & ctl_)
{
  const auto & name = ctl_.robot().name();
  output(ctl_.robot().name());
  return true;
}

EXPORT_SINGLE_STATE("ChooseRobot", ChooseRobot)
