#pragma once

#include <mc_control/fsm/State.h>
#include <fstream>


struct ChooseRobot : mc_control::fsm::State
{

    void configure(const mc_rtc::Configuration & config) override {}

    inline void start(mc_control::fsm::Controller & ctl) override {}

    bool run(mc_control::fsm::Controller & ctl) override;

    inline void teardown(mc_control::fsm::Controller & ctl) override {}
};
