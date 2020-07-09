#include "CalibrationMotionLogging.h"
#include "../ForceSensorCalibration.h"
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/io_utils.h>
#include <boost/filesystem.hpp>

template <typename Derived>
std::string to_string(const Eigen::MatrixBase<Derived>& v, const std::string & delimiter)
{
  if(v.cols() > 1)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("to_string on Eigen types expect a vector, got a matrix of size {}x{}", v.rows(), v.cols());
  }
  if(v.rows() == 0)
  {
    return "";
  }
  std::string out = std::to_string(v(0));
  for (unsigned i = 1; i < v.size(); ++i)
  {
    out += delimiter + std::to_string(v(i));
  }
  return out;
}


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
  outputPath_ = "/tmp/calib-force-sensors-data-" + robot.name();

  // Attempt to create the output files
  if(!boost::filesystem::exists(outputPath_))
  {
    if(!boost::filesystem::create_directory(outputPath_))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[CalibrationMotionLogging] Could not create output folder {}", outputPath_);
    }
  }
}


bool CalibrationMotionLogging::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ForceSensorCalibration &>(ctl_);

  auto & robot = ctl_.robot();
  // Log RPY
  loggers_["rpy"] << dt_ << " " << to_string(mc_rbdyn::rpyFromQuat(robot.bodySensor().orientation()), " ") << "\n";
  // Log q
  loggers_["q"] << dt_ << " " << mc_rtc::io::to_string(robot.encoderValues(), " ") << "\n";
  // Log sensors
  for(const auto & sensor: sensors_)
  {
    const auto & sensorName = sensor.first;
    const auto & sensorAlias = sensor.second;
    loggers_[sensorAlias] << dt_ << " "
        << to_string(robot.forceSensor(sensorName).force(), " ") << " "
        << to_string(robot.forceSensor(sensorName).couple(), " ") << "\n";
  }


  dt_ += ctl_.timeStep;
  output("OK");
  return true;
}

void CalibrationMotionLogging::teardown(mc_control::fsm::Controller &)
{
  mc_rtc::log::info("[CalibrationMotionLogging] Saving calibration data to {}");
  // Save all logger's output to file
  for(const auto & logger : loggers_)
  {
    const auto filename = outputPath_ + "/calib-force-sensors." + logger.first;
    std::ofstream file(filename);
    if(file)
    {
      file << logger.second.str();
      file.close();
      mc_rtc::log::info("[CalibrationMotionLogging] Calibration file {} written successfully.", filename);
    }
    else
    {
      mc_rtc::log::error("Could not write logging information for {} to file {}", logger.first, filename);
    }
  }
}

EXPORT_SINGLE_STATE("CalibrationMotionLogging", CalibrationMotionLogging)
