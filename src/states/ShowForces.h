#pragma once

#include <mc_control/fsm/State.h>
#include <mc_rtc/gui/types.h>

namespace mc_rbdyn
{
struct Robot;
struct ForceSensor;
} // namespace mc_rbdyn

namespace mc_rtc
{
namespace gui
{
struct StateBuilder;
} // namespace gui
} // namespace mc_rtc

namespace mc_control
{
namespace fsm
{
struct Controller;
} // namespace fsm
} // namespace mc_control

struct ShowForces : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void saveCalibration(mc_control::fsm::Controller & ctl);

  void addWrenchPlot(const std::string & name, mc_rtc::gui::StateBuilder & gui, const mc_rbdyn::ForceSensor & fs);
  void addWrenchWithoutGravityPlot(const std::string & name,
                                   mc_rtc::gui::StateBuilder & gui,
                                   const mc_rbdyn::Robot & robot,
                                   const mc_rbdyn::ForceSensor & fs);
  void addWrenchWithoutGravityPlot(const std::string & name,
                                   const std::string & surface,
                                   mc_rtc::gui::StateBuilder & gui,
                                   const mc_rbdyn::Robot & robot,
                                   const mc_rbdyn::ForceSensor & fs);

  void addWrenchVector(const std::string & name,
                       mc_rtc::gui::StateBuilder & gui,
                       const mc_rbdyn::Robot & robot,
                       const mc_rbdyn::ForceSensor & fs);
  void addWrenchWithoutGravityVector(const std::string & name,
                                     mc_rtc::gui::StateBuilder & gui,
                                     const mc_rbdyn::Robot & robot,
                                     const mc_rbdyn::ForceSensor & fs);
  void addWrenchWithoutGravityVector(const std::string & name,
                                     const std::string & surface,
                                     mc_rtc::gui::StateBuilder & gui,
                                     const mc_rbdyn::Robot & robot,
                                     const mc_rbdyn::ForceSensor & fs);

private:
  double t_ = 0;
  std::vector<std::pair<std::string, std::string>> sensors_;
  bool stop_ = false;

  std::vector<std::string> category_ = {"Calibration", "Show Forces"};
  std::vector<std::string> plots_;
  // Map of force sensor to surface name (chosen in the gui)
  std::map<std::string, std::string> surfaces_;
  double forceScale_ = 1;
  mc_rtc::gui::ForceConfig forceConfig_;
};
