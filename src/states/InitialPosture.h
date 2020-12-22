#pragma once

#include <mc_control/CompletionCriteria.h>
#include <mc_control/fsm/State.h>
#include <fstream>

struct InitialPosture : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override {}

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  inline void teardown(mc_control::fsm::Controller & ctl) override {}

private:
  double savedStiffness_ = 1;
  mc_control::CompletionCriteria crit_;
};
