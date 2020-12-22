#include "CheckResults.h"
#include <mc_rtc/gui/plot.h>
#include <boost/filesystem.hpp>
#include "../ForceSensorCalibration.h"

namespace bfs = boost::filesystem;

void CheckResults::configure(const mc_rtc::Configuration & config)
{
  config("checkDefault", checkDefault_);
}

void CheckResults::start(mc_control::fsm::Controller & ctl)
{
  auto & robot = ctl.robot();
  using namespace mc_rtc::gui;
  using Style = mc_rtc::gui::plot::Style;

  const auto & robotConf = ctl.config()(robot.name());
  if(!robotConf.has("forceSensors"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Calibration controller expects a forceSensors entry",
                                                     name());
  }
  sensors_ = robotConf("forceSensors");
  double duration = robotConf("motion")("duration", 30);

  std::string calib_path = "";
  if(checkDefault_)
  {
    calib_path = ctl.robot().module().calib_dir;
  }
  else
  {
    calib_path = "/tmp/calib-force-sensors-result-" + ctl.robot().name();
  }

  // Load new calibration parameters
  for(const auto & sensorN : sensors_)
  {
    const auto filename = calib_path + "/calib_data." + sensorN;
    mc_rtc::log::info("[{}] Loading calibration file {}", name(), filename);
    auto & sensor = ctl.robot().forceSensor(sensorN);
    sensor.loadCalibrator(filename, ctl.robot().mbc().gravity);

    ctl.logger().addLogEntry(sensor.name() + "_calibrated",
                             [&robot, &sensor]() { return sensor.wrenchWithoutGravity(robot); });

    ctl.gui()->addPlot(
        sensorN, plot::X({"t", {t_ + 0, t_ + duration}}, [this]() { return t_; }),
        plot::Y("Wrenches calibrated (x)",
                [&robot, &sensor]() { return sensor.wrenchWithoutGravity(robot).force().x(); }, Color::Red,
                Style::Solid),
        plot::Y("Wrenches calibrated (y)",
                [&robot, &sensor]() { return sensor.wrenchWithoutGravity(robot).force().y(); }, Color::Green,
                Style::Solid),
        plot::Y("Wrenches calibrated (y)",
                [&robot, &sensor]() { return sensor.wrenchWithoutGravity(robot).force().z(); }, Color::Blue,
                Style::Solid),
        plot::Y("Wrenches raw(x)", [&sensor]() { return sensor.wrench().force().x(); }, Color::Red, Style::Dashed),
        plot::Y("Wrenches raw(y)", [&sensor]() { return sensor.wrench().force().y(); }, Color::Green, Style::Dashed),
        plot::Y("Wrenches raw(z)", [&sensor]() { return sensor.wrench().force().z(); }, Color::Blue, Style::Dashed));
  }

  ctl.gui()->addElement(
      {}, Label("Status", []() { return "Check the plots to see if the calibrated measurements are close to zero"; }),
      Button("Save calibration", [this, &ctl]() { saveCalibration(ctl); }),
      Button("Finish without saving", [this, &ctl]() { running_ = false; }), Button("Save and finish", [this, &ctl]() {
        saveCalibration(ctl);
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
  else
  {
    if(ctl_.datastore().has("CalibrationMotion::Stop"))
    {
      ctl_.datastore().call("CalibrationMotion::Stop");
    }
  }
  output("OK");
  return true;
}

void CheckResults::saveCalibration(mc_control::fsm::Controller & ctl)
{
  mc_rtc::log::info("[ForceSensorCalibration] Saving calibration results");
  const auto & calib_dir = ctl.robot().module().calib_dir;
  if(!bfs::exists(calib_dir))
  {
    if(bfs::create_directories(calib_dir))
    {
      mc_rtc::log::info("[{}] Created missing calibration directory {}", name(), calib_dir);
    }
    else
    {
      mc_rtc::log::error("[{}] Failed to create calibration directory {}", name(), calib_dir);
      return;
    }
  }

  for(const auto & sensor : sensors_)
  {
    const auto source_path =
        "/tmp/calib-force-sensors-result-" + ctl.robot().name() + "/" + std::string("calib_data." + sensor);
    const auto destination_path = calib_dir + "/" + std::string("calib_data." + sensor);
    try
    {
      bfs::copy_file(source_path, destination_path, bfs::copy_option::overwrite_if_exists);
      mc_rtc::log::success("[ForceSensorCalibration] Calibration file copied to {}", destination_path);
    }
    catch(...)
    {
      mc_rtc::log::warning("[ForceSensorCalibration] Failed to save {} calibration file to {}", sensor,
                           destination_path);
    }
  }
}

void CheckResults::teardown(mc_control::fsm::Controller & ctl)
{
  for(const auto & sensor : sensors_)
  {
    ctl.gui()->removePlot(sensor);
    ctl.gui()->removeElement({}, "Status");
    ctl.gui()->removeElement({}, "Save calibration");
    ctl.gui()->removeElement({}, "Finish without saving");
    ctl.gui()->removeElement({}, "Save and finish");
    ctl.logger().removeLogEntry(sensor + "_calibrated");
  }
}

EXPORT_SINGLE_STATE("CheckResults", CheckResults)
