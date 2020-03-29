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
    LOG_ERROR_AND_THROW(std::runtime_error, "to_string on Eigen types expect a vector, got a matrix of size " << v.rows() << "x" << v.cols());
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
    LOG_ERROR_AND_THROW(std::runtime_error, "Calibration controller expects a forceSensors entry");
  }
  sensors_ = robotConf("forceSensors");
  outputPath_ = "/tmp/calib-force-sensors-data-" + robot.name();

  // Attempt to create the output files
  if(!boost::filesystem::exists(outputPath_))
  {
    if(!boost::filesystem::create_directory(outputPath_))
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "[CalibrationMotionLogging] Could not create output folder " << outputPath_);
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

void CalibrationMotionLogging::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ForceSensorCalibration &>(ctl_);

  LOG_INFO("[CalibrationMotionLogging] Saving calibration data to " << outputPath_);
  // Save all logger's output to file
  for(const auto & logger : loggers_)
  {
    const auto filename = outputPath_ + "/calib-force-sensors." + logger.first;
    std::ofstream file(filename);
    if(file)
    {
      file << logger.second.str();
      file.close();
      LOG_INFO("[CalibrationMotionLogging] Calibration file " << filename << " written successfully.");
    }
    else
    {
      LOG_ERROR("Could not write logging information for " << logger.first << " to file: " << filename);
    }
  }
}

EXPORT_SINGLE_STATE("CalibrationMotionLogging", CalibrationMotionLogging)
