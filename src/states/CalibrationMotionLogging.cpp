#include "CalibrationMotionLogging.h"

#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/io_utils.h>

#include <boost/filesystem.hpp>

#include "../ForceSensorCalibration.h"
#include "../Measurement.h"

namespace bfs = boost::filesystem;

void CalibrationMotionLogging::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ForceSensorCalibration &>(ctl_);

  auto & robot = ctl.robot();
  auto robotConf = ctl.config()(robot.name());

  singularityThreshold_ = ctl.config()("SingularityThreshold");

  if(!robotConf.has("forceSensors"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Calibration controller expects a forceSensors entry");
  }
  sensors_ = robotConf("forceSensors");
  auto outputPath_ = (bfs::temp_directory_path() / fmt::format("calib-force-sensors-data-{}", robot.name())).string();

  // Attempt to create the output files
  if(!boost::filesystem::exists(outputPath_))
  {
    if(!boost::filesystem::create_directory(outputPath_))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Could not create output folder {}", name(), outputPath_);
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
    measurements[s] = {};
    measurementsCount_[s] = {0, 0};
    if(singularityThreshold_ > 0)
    {
      const auto & sensor = ctl_.robot().forceSensor(s);
      jacobians_[s] = {ctl_.robot(), sensor.parentBody()};
    }
  }
}

bool CalibrationMotionLogging::run(mc_control::fsm::Controller & ctl_)
{
  auto & robot = ctl_.robot();
  auto & real = ctl_.realRobot();
  auto & measurements = ctl_.datastore().get<SensorMeasurements>("measurements");
  for(const auto & s : sensors_)
  {
    const auto & sensor = robot.forceSensor(s);
    const auto & X_0_p = real.bodyPosW()[real.bodyIndexByName(sensor.parentBody())];
    const auto & measure = sensor.wrench();
    measurementsCount_[s].second++;
    if(singularityThreshold_ > 0)
    {
      auto & j = jacobians_[s];
      const auto & jacMat = j.jacobian.jacobian(robot.mb(), robot.mbc());
      j.svd.compute(jacMat);
      if(j.svd.singularValues().tail(1)(0) > singularityThreshold_)
      {
        measurements[s].push_back({X_0_p, measure});
        measurementsCount_[s].first++;
      }
    }
    else
    {
      measurements[s].push_back({X_0_p, measure});
      measurementsCount_[s].first++;
    }
  }
  output("OK");
  return true;
}

void CalibrationMotionLogging::teardown(mc_control::fsm::Controller &)
{
  for(const auto & m : measurementsCount_)
  {
    mc_rtc::log::info("[{}] {} records taken out of {} iterations", m.first, m.second.first, m.second.second);
  }
}

EXPORT_SINGLE_STATE("CalibrationMotionLogging", CalibrationMotionLogging)
