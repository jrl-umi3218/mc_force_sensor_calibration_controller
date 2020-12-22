#pragma once

#include <mc_control/fsm/State.h>

struct CalibrationMotion : mc_control::fsm::State
{

  inline void configure(const mc_rtc::Configuration & config) override {}

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  std::vector<std::function<void()>> jointUpdates_;
  double dt_ = 0;
  double duration_ = 60;
  double percentLimits_ = 0.8;
  double savedStiffness_ = 10;
  bool interrupted_ = false;
};
