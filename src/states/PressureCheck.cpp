#include "PressureCheck.h"
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/io_utils.h>
#include <boost/filesystem.hpp>
#include "../ForceSensorCalibration.h"

#include <mc_rtc/gui.h>

void PressureCheck::start(mc_control::fsm::Controller & ctl)
{
  const auto & robotConf = ctl.config()(ctl.robot().name());
  robotConf("maxPressureThreshold", maxPressure_);
  robotConf("forceSensors", forceSensors_);

  check(ctl);
  output("OK");
}

void PressureCheck::check(mc_control::fsm::Controller & ctl)
{
  success_ = true;
  std::string error = "";
  std::vector<std::string> errorSensors;
  for(const auto & sensor : forceSensors_)
  {
    auto force = ctl.robot().forceSensor(sensor.first).force().norm();
    if(force > maxPressure_)
    {
      error += fmt::format("Excessive force on sensor {} (force (norm) {} > maxForceThreshold {})\n", sensor.first,
                           force, maxPressure_);
      errorSensors.push_back(sensor.first);
      success_ = false;
    }
  }
  if(!success_)
  {
    mc_rtc::log::error("[{}] Excessive force detected on sensors [{}]:\n{}\nPlease make sure that they are not in "
                       "contact and that the calibration motion can be safely executed, then click on \"Continue\".",
                       name(), mc_rtc::io::to_string(errorSensors), error);
    ctl.gui()->removeElement({}, "Error");
    ctl.gui()->removeElement({}, "Continue");
    ctl.gui()->addElement(
        {}, mc_rtc::gui::Label("Error", [errorSensors]() {
          return fmt::format("Excessive force detected on sensors [{}], please make sure that they are not in contact "
                             "and that the calibration motion can be safely executed, then click on \"Continue\".",
                             mc_rtc::io::to_string(errorSensors));
        }));
    ctl.gui()->addElement({}, mc_rtc::gui::Button("Continue", [this, &ctl]() { check(ctl); }));
  }
}

bool PressureCheck::run(mc_control::fsm::Controller & ctl)
{
  return success_;
}

void PressureCheck::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeElement({}, "Error");
  ctl.gui()->removeElement({}, "Continue");
}

EXPORT_SINGLE_STATE("PressureCheck", PressureCheck)
