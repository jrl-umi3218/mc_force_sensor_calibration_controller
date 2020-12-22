#pragma once

#include <mc_control/fsm/State.h>

struct CheckResults : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void saveCalibration(mc_control::fsm::Controller & ctl);

private:
  double t_ = 0;
  std::vector<std::string> sensors_;
  bool running_ = true;
  bool checkDefault_ = false;
};
