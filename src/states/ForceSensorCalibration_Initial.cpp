#include "ForceSensorCalibration_Initial.h"

#include "../ForceSensorCalibration.h"

void ForceSensorCalibration_Initial::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
}

void ForceSensorCalibration_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ForceSensorCalibration &>(ctl_);

  auto & robot = ctl.robot();


  if(!config_.has("joints"))
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "Calibration controller expects a joints entry");
  }
  config_("duration", duration_);
  config_("stiffness", stiffness_);
  ctl_.getPostureTask(ctl_.robot().name())->stiffness(stiffness_);

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

bool ForceSensorCalibration_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ForceSensorCalibration &>(ctl_);

  // Update all joint positions
  for(auto & updateJoint : jointUpdates_)
  {
    updateJoint();
  }

  dt_ += ctl_.timeStep;
  if(dt_ > duration_)
  {
    output("OK");
    return true;
  }
  return false;
}

void ForceSensorCalibration_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ForceSensorCalibration &>(ctl_);
}

EXPORT_SINGLE_STATE("ForceSensorCalibration_Initial", ForceSensorCalibration_Initial)
