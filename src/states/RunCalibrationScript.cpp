#include "RunCalibrationScript.h"
#include "../ForceSensorCalibration.h"
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

  th_ = std::thread(
      [this,&ctl_]()
      {
        std::string command = "~/src/force_sensors_calibration/calib_force.py --robot HRP2DRC --dry-run /tmp/calib-force-sensors-data-"+ctl_.robot().name()+" --dry-run-path " + outputPath_ + " wrist";
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
