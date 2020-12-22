#pragma once

#include <mc_control/fsm/State.h>
#include <atomic>
#include <fstream>
#include <thread>

struct RunCalibrationScript : mc_control::fsm::State
{

  ~RunCalibrationScript() override;

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  std::thread th_;
  std::atomic<bool> completed_{false};
  bool success_ = true;
  std::string outputPath_;
  std::vector<std::string> sensors_;
};
