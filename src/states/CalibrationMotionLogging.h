#pragma once

#include <mc_control/fsm/State.h>
#include <fstream>

struct CalibrationMotionLogging : mc_control::fsm::State
{

    inline void configure(const mc_rtc::Configuration & config) override {}

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

private:
    // pair of sensor (name, logging alias) */
    std::vector<std::pair<std::string, std::string>> sensors_;
};
