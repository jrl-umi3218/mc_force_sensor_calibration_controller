#include "ForceSensorCalibration.h"

#include <mc_rbdyn/configuration_io.h>
#include <mc_rtc/io_utils.h>

namespace
{

mc_rtc::Configuration patchConfig(const mc_rbdyn::RobotModule & rm, const mc_rtc::Configuration & config)
{
  auto out = config;
  if(!config.has(rm.name))
  {
    return out;
  }
  auto rConfig = config(rm.name);
  if(rConfig.has("SingularityThreshold"))
  {
    out.add("SingularityThreshold", rConfig("SingularityThreshold"));
  }
  return out;
}

} // namespace

ForceSensorCalibration::ForceSensorCalibration(mc_rbdyn::RobotModulePtr rm,
                                               double dt,
                                               const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, patchConfig(*rm, config))
{
  if(!config.has(rm->name))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "No configuration section for {}, add a configuration for this robot before attempting calibration", rm->name);
  }
  auto rConfig = config(rm->name);
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
  if(config()(robot().name()).has("collisions"))
  {
    addCollisions(robot().name(), "ground", config()(robot().name())("collisions"));
  }
}
