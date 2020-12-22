#include "InitialPosture.h"
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/io_utils.h>
#include <boost/filesystem.hpp>
#include "../ForceSensorCalibration.h"

void InitialPosture::start(mc_control::fsm::Controller & ctl)
{
  const auto & robotConf = ctl.config()(ctl.robot().name());
  if(!robotConf.has("initial_posture"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Calibration controller expects an initial_posture entry",
                                                     name());
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
