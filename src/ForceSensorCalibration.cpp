#include "ForceSensorCalibration.h"

#include <mc_rbdyn/configuration_io.h>
#include <mc_rtc/io_utils.h>

ForceSensorCalibration::ForceSensorCalibration(mc_rbdyn::RobotModulePtr rm,
                                               double dt,
                                               const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  auto robotsConfig = config("robots");
  if(!robotsConfig.has(rm->name))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "No configuration section for {}, add a configuration for this robot before attempting calibration", rm->name);
  }
  auto rConfig = robotsConfig(rm->name);
  mc_rtc::log::info("rConfig:\n{}", rConfig.dump(true, true));
  if(rConfig.has("ObserverPipelines"))
  {
    MCController::createObserverPipelines(rConfig);
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "This controller requires at least an encoder observer for robot {}. In addition, floating base robots also "
        "require estimation of the floating base orientation.",
        robot().name());
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
  auto rConfig = config()("robots")(robot().name());
  if(rConfig.has("collisions"))
  {
    addCollisions(robot().name(), "ground", rConfig("collisions"));
  }
}
