#include "InitialPosture.h"
#include "../ForceSensorCalibration.h"
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/io_utils.h>
#include <boost/filesystem.hpp>

void InitialPosture::start(mc_control::fsm::Controller & ctl)
{
  const auto & robotConf = ctl.config()(ctl.robot().name());
  if(!robotConf.has("initial_posture"))
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "Calibration controller expects an initial_posture entry");
  }
  const auto & conf(robotConf("initial_posture"));
  auto postureTask = ctl.getPostureTask(ctl.robot().name());
  postureTask->target(conf("target"));
  savedStiffness_ = postureTask->stiffness();
  postureTask->stiffness(conf("stiffness", 10));
  crit_.configure(*postureTask, ctl.solver().dt(), conf("completion", mc_rtc::Configuration{}));
}

bool InitialPosture::run(mc_control::fsm::Controller & ctl_)
{
  auto postureTask = ctl_.getPostureTask(ctl_.robot().name());
  if(crit_.completed(*postureTask))
  {
    output("OK");
    return true;
  }
  return false;
}

EXPORT_SINGLE_STATE("InitialPosture", InitialPosture)