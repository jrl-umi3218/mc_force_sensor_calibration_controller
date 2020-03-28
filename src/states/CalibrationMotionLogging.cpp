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


void CalibrationMotionLogging::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
}

void CalibrationMotionLogging::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ForceSensorCalibration &>(ctl_);

  auto & robot = ctl.robot();


  if(!config_.has("joints"))
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "Calibration controller expects a joints entry");
  }
  if(!config_.has("forceSensors"))
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "Calibration controller expects a forceSensors entry");
  }
  sensors_ = config_("forceSensors");
  config_("duration", duration_);
  config_("outputPath", outputPath_);

  // Attempt to create the output files
  if(!boost::filesystem::exists(outputPath_))
  {
    if(!boost::filesystem::create_directory(outputPath_))
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "[CalibrationMotionLogging] Could not create output folder " << outputPath_);
    }
  }


  auto postureTask = ctl_.getPostureTask(ctl_.robot().name());
  savedStiffness_ = postureTask->stiffness();
  postureTask->stiffness(config_("stiffness", 10));
  constexpr double PI = mc_rtc::constants::PI;
  for(const auto & jConfig : config_("joints"))
  {
    std::string name = jConfig("name");
    double period = jConfig("period");
    auto jidx = robot.jointIndexByName(name);
    auto start = robot.mbc().q[jidx][0];
    auto lower = robot.ql()[jidx][0];
    auto upper = robot.qu()[jidx][0];
    // compute the starting time such that the joint does not move initially
    // that is such that f(start_dt) = start
    // i.e start_dt = f^(-1)(start)
    double start_dt = period * (acos(sqrt(start - lower)/sqrt(upper-lower))) / PI;
    jointUpdates_.emplace_back(
      /* f(t): periodic function that moves the joint between its limits */
      [this, &ctl_, PI, start, lower, upper, start_dt, period, name]()
      {
        auto t = start_dt + dt_;
        auto q = lower + (upper-lower) * (1 + cos((2*PI*t)/period)) / 2;
        ctl_.getPostureTask(ctl_.robot().name())->target({{name, {q}}});
      }
    );
  }
}


bool CalibrationMotionLogging::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ForceSensorCalibration &>(ctl_);

  // Update all joint positions
  for(auto & updateJoint : jointUpdates_)
  {
    updateJoint();
  }

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
  if(dt_ > duration_)
  {
    output("OK");
    return true;
  }
  return false;
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

  auto postureTask = ctl_.getPostureTask(ctl_.robot().name());
  postureTask->stiffness(savedStiffness_);
}

EXPORT_SINGLE_STATE("CalibrationMotionLogging", CalibrationMotionLogging)
