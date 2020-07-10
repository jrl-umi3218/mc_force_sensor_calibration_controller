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
  if(rConfig.has("RunObservers"))
  {
    out.add("RunObservers", rConfig("RunObservers"));
  }
  if(rConfig.has("UpdateObservers"))
  {
    out.add("UpdateObservers", rConfig("UpdateObservers"));
  }
  return out;
}

}

ForceSensorCalibration::ForceSensorCalibration(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  if(!config.has(rm->name))
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
  if(config()(robot().name()).has("collisions"))
  {
    addCollisions(robot().name(), "ground", config()(robot().name())("collisions"));
  }
}


bool ForceSensorCalibration::resetObservers()
{
  // Change the list of observers if specified per-robot
  const auto & rConf = config()(robot().name());
  auto runObservers = rConf("RunObservers", std::vector<std::string>{});
  auto updateObservers = rConf("UpdateObservers", std::vector<std::string>{});
  if(runObservers.empty())
  {
    mc_rtc::log::info("[ForceSensorCalibration] No custom observer pipeline specified for robot {}, using the default pipeline.", robot().name());
  }
  else
  {
    mc_rtc::log::info("[ForceSensorCalibration] Custom pipeline specified for robot {}", robot().name());
    pipelineObservers_.clear();
    for(const auto & observerName : runObservers)
    {
      auto observerIt = std::find_if(observers_.begin(), observers_.end(), [&observerName](const mc_observers::ObserverPtr & obs)
                                  {
                                  return obs->name() == observerName;
                                  });
      if(observerIt != observers_.end())
      {
        bool update = std::find(updateObservers.begin(), updateObservers.end(), observerName) != updateObservers.end();
        pipelineObservers_.emplace_back(*observerIt, update);
      }
      else
      {
        mc_rtc::log::error_and_throw<std::runtime_error>("[ForceSensorCalibration] Requested observer {} but this observer hasn't been loaded (loaded observers are [{}]",
                                                         observerName,
                                                         mc_rtc::io::to_string(observers_, [](const auto & obs) { return obs->name(); }));
      }
    }
  }
  return MCController::resetObservers();
}
