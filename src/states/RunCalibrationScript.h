#pragma once

#include <mc_control/fsm/State.h>
#include <fstream>
#include <atomic>
#include <thread>

struct RunCalibrationScript : mc_control::fsm::State
{

    ~RunCalibrationScript();

    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

private:
    std::thread th_;
    std::atomic<bool> completed_{false};
    bool success_ = false;
    std::string outputPath_;
};
