#pragma once

#include <mc_control/fsm/State.h>


struct ForceSensorCalibration_Initial : mc_control::fsm::State
{

    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;
private:
    mc_rtc::Configuration config_;
    std::vector<std::function<void()>> jointUpdates_;
    double dt_ = 0;
    double duration_ = 60;
    double stiffness_ = 10;
};
