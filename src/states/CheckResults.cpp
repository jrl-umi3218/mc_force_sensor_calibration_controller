#include "CheckResults.h"
#include "../ForceSensorCalibration.h"
#include <mc_rtc/gui/plot.h>
#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem;


void CheckResults::start(mc_control::fsm::Controller & ctl)
{
  auto & robot = ctl.robot();
  using namespace mc_rtc::gui;
  using Style = mc_rtc::gui::plot::Style;

  const auto & robotConf = ctl.config()(robot.name());
  if(!robotConf.has("forceSensors"))
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "Calibration controller expects a forceSensors entry");
  }
  sensors_ = robotConf("forceSensors");
  double duration = robotConf("motion")("duration", 30);

  // Load new calibration parameters
  for(const auto & sensorP : sensors_)
  {
    auto sensor = sensorP.first;
    const auto filename = "/tmp/calib-force-sensors-result-"+ctl.robot().name()+"/calib_data."+sensor;
    LOG_INFO("[ForceSensorCalibration] Loading calibration file " << filename);
    ctl.robot().forceSensor(sensor).loadCalibrator(filename, ctl.robot().mbc().gravity);

    ctl.gui()->addPlot(sensor,
                   plot::X({"t", {t_ + 0, t_ + duration}}, [this]() { return t_; }),
                   plot::Y("Wrenches calibrated (x)", [this, &robot]() { return robot.forceSensor("RightHandForceSensor").wrenchWithoutGravity(robot).force().x(); }, Color::Red, Style::Solid),
                   plot::Y("Wrenches calibrated (y)", [this, &robot]() { return robot.forceSensor("RightHandForceSensor").wrenchWithoutGravity(robot).force().y(); }, Color::Green, Style::Solid),
                   plot::Y("Wrenches calibrated (y)", [this, &robot]() { return robot.forceSensor("RightHandForceSensor").wrenchWithoutGravity(robot).force().z(); }, Color::Blue, Style::Solid),
                   plot::Y("Wrenches raw(x)", [this, &robot]() { return robot.forceSensor("RightHandForceSensor").wrench().force().x(); }, Color::Red, Style::Dashed),
                   plot::Y("Wrenches raw(y)", [this, &robot]() { return robot.forceSensor("RightHandForceSensor").wrench().force().y(); }, Color::Green, Style::Dashed),
                   plot::Y("Wrenches raw(z)", [this, &robot]() { return robot.forceSensor("RightHandForceSensor").wrench().force().z(); }, Color::Blue, Style::Dashed)
                   );
  }

  ctl.gui()->addElement({},
      Label("Status",
            []()
            {
              return "Check the plots to see if the calibrated measurements are close to zero";
            }),
      Button("Save calibration",
             [this, &ctl]()
             {
              saveCalibration(ctl);
             }),
      Button("Stop (without saving)",
             [this]
             {
              running_ = false;
             }));
}

bool CheckResults::run(mc_control::fsm::Controller & ctl_)
{
  if(running_)
  {
    t_ += ctl_.timeStep;
    return false;
  }
  output("OK");
  return true;
}

void CheckResults::saveCalibration(mc_control::fsm::Controller & ctl)
{
  for(const auto & sensorP : sensors_)
  {
    const auto & sensor = sensorP.first;
    const auto source_path = "/tmp/calib-force-sensors-result-"+ctl.robot().name() + "/" + std::string("calib_data." + sensor);
    const auto destination_path = ctl.robot().module().calib_dir + "/" + std::string("calib_data." + sensor);
    try
    {
      bfs::copy_file(source_path,destination_path, bfs::copy_option::overwrite_if_exists);
      LOG_SUCCESS("[ForceSensorCalibration] Calibration file copied to " << destination_path);
    }
    catch(...)
    {
      LOG_WARNING("[ForceSensorCalibration] Failed to save " << sensor << " calibration file to " << destination_path);
    }
  }
}


void CheckResults::teardown(mc_control::fsm::Controller & ctl)
{
  for(const auto & sensorP : sensors_)
  {
    const auto & sensor = sensorP.first;
    ctl.gui()->removePlot(sensor);
    ctl.gui()->removeElement({}, "Status");
    ctl.gui()->removeElement({}, "Save calibration");
  }
}

EXPORT_SINGLE_STATE("CheckResults", CheckResults)
