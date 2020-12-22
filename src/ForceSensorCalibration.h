#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>

#include "api.h"

struct ForceSensorCalibration_DLLAPI ForceSensorCalibration : public mc_control::fsm::Controller
{
  ForceSensorCalibration(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  /**
   * Do not create observers from global configuration.
   * They will be created in the constructor according to the per-robot
   * configuration instead
   */
  void createObserverPipelines(const mc_rtc::Configuration & config) override {}

private:
  mc_rtc::Configuration config_;
};
