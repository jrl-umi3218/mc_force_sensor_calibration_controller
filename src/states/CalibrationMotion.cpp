#include "CalibrationMotion.h"
#include "../ForceSensorCalibration.h"


void CalibrationMotion::start(mc_control::fsm::Controller & ctl)
{
  auto & robot = ctl.robot();
  auto robotConf = ctl.config()(robot.name());
  if(!robotConf.has("motion"))
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "Calibration controller expects a joints entry");
  }
  auto conf = robotConf("motion");
  conf("duration", duration_);

  auto postureTask = ctl.getPostureTask(robot.name());
  savedStiffness_ = postureTask->stiffness();
  postureTask->stiffness(conf("stiffness", 10));
  constexpr double PI = mc_rtc::constants::PI;
  for(const auto & jConfig : conf("joints"))
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
      [this, postureTask, PI, start, lower, upper, start_dt, period, name]()
      {
        auto t = start_dt + dt_;
        auto q = lower + (upper-lower) * (1 + cos((2*PI*t)/period)) / 2;
        postureTask->target({{name, {q}}});
      }
    );
  }

  ctl.gui()->addElement({},
                         mc_rtc::gui::NumberSlider("Progress",
                                                   [this]()
                                                   {
                                                    return dt_;
                                                   },
                                                   [](double)
                                                   {
                                                   },
                                                   0,
                                                   duration_),
                         mc_rtc::gui::Button("Stop",
                                             [this]()
                                             {
                                              LOG_WARNING("[ForceSensorCalibration] Motion was interrupted before it's planned duration (" << dt_ << " / " << duration_ << ")");
                                              interrupted_ = true;
                                             }
                                            )
                         );
}


bool CalibrationMotion::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ForceSensorCalibration &>(ctl_);

  // Update all joint positions
  for(auto & updateJoint : jointUpdates_)
  {
    updateJoint();
  }

  dt_ += ctl_.timeStep;
  if(interrupted_ || dt_ > duration_)
  {
    output("OK");
    return true;
  }
  return false;
}

void CalibrationMotion::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ForceSensorCalibration &>(ctl_);

  auto postureTask = ctl_.getPostureTask(ctl_.robot().name());
  postureTask->stiffness(savedStiffness_);
  ctl_.gui()->removeElement({}, "Progress");
  ctl_.gui()->removeElement({}, "Stop");
}

EXPORT_SINGLE_STATE("CalibrationMotion", CalibrationMotion)
