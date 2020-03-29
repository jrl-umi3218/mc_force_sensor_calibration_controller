#include "RunCalibrationScript.h"
#include "../ForceSensorCalibration.h"
#include "../../build/src/config.h"
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/io_utils.h>
#include <boost/filesystem.hpp>

RunCalibrationScript::~RunCalibrationScript()
{
  th_.join();
}

void RunCalibrationScript::configure(const mc_rtc::Configuration & config)
{
}

void RunCalibrationScript::start(mc_control::fsm::Controller & ctl_)
{
  outputPath_ = "/tmp/calib-force-sensors-result-" + ctl_.robot().name();
  std::string moduleName = ctl_.config()("MainRobot");
  std::string robotName = ctl_.robot().name();
  bool showPlots = ctl_.config()("plots", false);

  th_ = std::thread(
      [this, showPlots, robotName, moduleName]()
      {
        std::string plots = showPlots ? " --show-plots " : "";
        std::string command = std::string(calib_config::CALIBRATION_SCRIPT_EXECUTABLE) + plots + " --robot " + moduleName + " --dry-run /tmp/calib-force-sensors-data-"+robotName+" --dry-run-path " + outputPath_ + " all";
        LOG_INFO("Executing calibration script: " << command);
        auto result = std::system(command.c_str());
        success_ = (result == 0);
        completed_ = true;
      });
  ctl_.gui()->addElement({},
                         mc_rtc::gui::Label("Status",
                                            []()
                                            {
                                            return "Calibration script running, this may take several minutes";
                                            })
                         );
}


bool RunCalibrationScript::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ForceSensorCalibration &>(ctl_);

  if(completed_)
  {
    if(!success_)
    {
      LOG_ERROR("[ForceSensor] Calibration script failed");
      output("FAILED");
      return true;
    }
    LOG_INFO("[ForceSensorCalibration] Calibration files written to " << outputPath_);
    output("SUCCESS");
    return true;
  }
  return false;
}

void RunCalibrationScript::teardown(mc_control::fsm::Controller & ctl_)
{
  ctl_.gui()->removeElement({}, "Status");
}

EXPORT_SINGLE_STATE("RunCalibrationScript", RunCalibrationScript)
