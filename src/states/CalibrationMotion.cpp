#include "CalibrationMotion.h"
#include <mc_filter/utils/clamp.h>
#include "../ForceSensorCalibration.h"

void CalibrationMotion::start(mc_control::fsm::Controller & ctl)
{
  ctl.datastore().make_call("CalibrationMotion::Stop", [this]() { interrupted_ = true; });
  auto & robot = ctl.robot();
  auto robotConf = ctl.config()(robot.name());
  if(!robotConf.has("motion"))
  {
    mc_rtc::log::error("[{}] Calibration controller expects a joints entry", name());
    output("FAILURE");
  }
  auto conf = robotConf("motion");
  conf("duration", duration_);
  conf("percentLimits", percentLimits_);
  mc_filter::utils::clampInPlace(percentLimits_, 0, 1);

  auto postureTask = ctl.getPostureTask(robot.name());
  savedStiffness_ = postureTask->stiffness();
  postureTask->stiffness(conf("stiffness", 10));
  constexpr double PI = mc_rtc::constants::PI;
  for(const auto & jConfig : conf("joints"))
  {
    std::string name = jConfig("name");
    if(!ctl.robot().hasJoint(name))
    {
      mc_rtc::log::error("[{}] No joint named \"{}\" in robot \"{}\"", this->name(), name, ctl.robot().name());
      output("FAILURE");
    }
    auto percentLimits = percentLimits_;
    jConfig("percentLimits", percentLimits);
    mc_filter::utils::clampInPlace(percentLimits, 0, 1);
    double period = jConfig("period");
    auto jidx = robot.jointIndexByName(name);
    auto start = robot.mbc().q[jidx][0];
    auto actualLower = robot.ql()[jidx][0];
    auto actualUpper = robot.qu()[jidx][0];
    auto actualRange = actualUpper - actualLower;

    // Reduced range
    const auto range = percentLimits * actualRange;
    const auto lower = actualLower + (actualRange - range) / 2;
    const auto upper = actualUpper - (actualRange - range) / 2;

    if(start < lower || start > upper)
    {
      mc_rtc::log::error("[{}] Starting joint configuration of joint {} [{}] is outside of the reduced limit range "
                         "[{}, {}] (percentLimits: {}, actual joint limits: [{}, {}]",
                         this->name(), name, start, lower, upper, percentLimits, actualLower, actualUpper);
      output("FAILURE");
    }

    // compute the starting time such that the joint does not move initially
    // that is such that f(start_dt) = start
    // i.e start_dt = f^(-1)(start)
    double start_dt = period * (acos(sqrt(start - lower) / sqrt(upper - lower))) / PI;
    jointUpdates_.emplace_back(
        /* f(t): periodic function that moves the joint between its limits */
        [this, postureTask, lower, upper, start_dt, period, name]() {
          auto t = start_dt + dt_;
          auto q = lower + (upper - lower) * (1 + cos((2 * PI * t) / period)) / 2;
          postureTask->target({{name, {q}}});
        });
  }

  ctl.gui()->addElement({},
                        mc_rtc::gui::NumberSlider(
                            "Progress", [this]() { return dt_; }, [](double) {}, 0, duration_),
                        mc_rtc::gui::Button("Stop Motion", [this]() {
                          mc_rtc::log::warning(
                              "[{}] Motion was interrupted before it's planned duration ({:.2f}/{:.2f}s)", name(), dt_,
                              duration_);
                          interrupted_ = true;
                        }));
}

bool CalibrationMotion::run(mc_control::fsm::Controller & ctl_)
{
  if(output() == "FAILURE")
  {
    return true;
  }

  // Update all joint positions
  for(auto & updateJoint : jointUpdates_)
  {
    updateJoint();
  }

  if(dt_ > duration_)
  {
    output("OK");
    return true;
  }
  else if(interrupted_)
  {
    output("INTERRUPTED");
    return true;
  }

  dt_ += ctl_.timeStep;
  return false;
}

void CalibrationMotion::teardown(mc_control::fsm::Controller & ctl_)
{
  auto postureTask = ctl_.getPostureTask(ctl_.robot().name());
  postureTask->stiffness(savedStiffness_);
  ctl_.gui()->removeElement({}, "Progress");
  ctl_.gui()->removeElement({}, "Stop Motion");
  ctl_.datastore().remove("CalibrationMotion::Stop");
}

EXPORT_SINGLE_STATE("CalibrationMotion", CalibrationMotion)
