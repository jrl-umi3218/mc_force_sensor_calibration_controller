#pragma once

#include <mc_control/fsm/State.h>

namespace mc_control
{
namespace fsm
{
struct Controller;
} // namespace fsm
} // namespace mc_control

struct ChooseTransition : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  std::map<std::string, std::string> actions_;
  std::vector<std::string> category_ = {"ChooseTransition"};
};
