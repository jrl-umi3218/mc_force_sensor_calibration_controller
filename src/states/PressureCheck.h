#pragma once

#include <mc_control/CompletionCriteria.h>
#include <mc_control/fsm/State.h>
#include <fstream>

struct PressureCheck : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override {}
  void start(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;

  void check(mc_control::fsm::Controller & ctl);

protected:
  bool success_ = false;
  double maxPressure_ = 50;
  std::vector<std::pair<std::string, std::string>> forceSensors_;
};
