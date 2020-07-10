#include "CalibrationMotionLogging.h"

#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/io_utils.h>

#include <boost/filesystem.hpp>

#include "../ForceSensorCalibration.h"
#include "../Measurement.h"

void CalibrationMotionLogging::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ForceSensorCalibration &>(ctl_);

  auto & robot = ctl.robot();
  auto robotConf = ctl.config()(robot.name());

  if(!robotConf.has("forceSensors"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Calibration controller expects a forceSensors entry");
  }
  sensors_ = robotConf("forceSensors");
  std::string outputPath_ = "/tmp/calib-force-sensors-data-" + robot.name();

  // Attempt to create the output files
  if(!boost::filesystem::exists(outputPath_))
  {
    if(!boost::filesystem::create_directory(outputPath_))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[CalibrationMotionLogging] Could not create output folder {}", outputPath_);
    }
  }

  if(!ctl.datastore().has("measurements"))
  {
    ctl.datastore().make<SensorMeasurements>("measurements");
  }
  auto & measurements = ctl.datastore().get<SensorMeasurements>("measurements");
  measurements.clear();
  for(const auto & s : sensors_)
  {
    measurements[s.first] = {};
    const auto & sensor = ctl_.robot().forceSensor(s.first);
    jacobians_[s.first] = {ctl_.robot(), sensor.parentBody()};
  }
}


bool CalibrationMotionLogging::run(mc_control::fsm::Controller & ctl_)
{
  auto & robot = ctl_.robot();
  auto & real = ctl_.realRobot();
  auto & measurements = ctl_.datastore().get<SensorMeasurements>("measurements");
  for(const auto & s : sensors_)
  {
    const auto & sensor = robot.forceSensor(s.first);
    const auto & X_0_p = real.bodyPosW()[real.bodyIndexByName(sensor.parentBody())];
    const auto & measure = sensor.wrench();
    auto & j = jacobians_[s.first];
    const auto & jacMat = j.jacobian.jacobian(robot.mb(), robot.mbc());
    j.svd.compute(jacMat);
    if(j.svd.singularValues().tail(1)(0) > 0.08)
    {
      measurements[s.first].push_back({X_0_p, measure});
    }
  }
  output("OK");
  return true;
}

void CalibrationMotionLogging::teardown(mc_control::fsm::Controller &)
{
}

EXPORT_SINGLE_STATE("CalibrationMotionLogging", CalibrationMotionLogging)
