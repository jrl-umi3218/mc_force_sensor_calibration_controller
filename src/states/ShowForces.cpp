#include "ShowForces.h"
#include <mc_control/fsm/Controller.h>
#include <mc_rbdyn/Robot.h>
#include <mc_rtc/gui.h>

using namespace mc_rtc::gui;
using Style = mc_rtc::gui::plot::Style;

void ShowForces::configure(const mc_rtc::Configuration & config)
{
  config("category", category_);
}


void ShowForces::addWrenchPlot(const std::string & name, mc_rtc::gui::StateBuilder & gui, const mc_rbdyn::ForceSensor & fs)
{
  gui.addPlot(name,
    plot::X("t", [this]() { return t_; }),
    plot::Y(name+ " (x)", [&fs]() { return fs.wrench().force().x(); }, Color::Red, Style::Dashed),
    plot::Y(name+ " (y)", [&fs]() { return fs.wrench().force().y(); }, Color::Green, Style::Dashed),
    plot::Y(name+ " (z)", [&fs]() { return fs.wrench().force().z(); }, Color::Blue, Style::Dashed));
  plots_.push_back(name);
}

void ShowForces::addWrenchWithoutGravityPlot(const std::string & name, mc_rtc::gui::StateBuilder & gui, const mc_rbdyn::Robot & robot, const mc_rbdyn::ForceSensor & fs)
{
  gui.addPlot(name,
    plot::X("t", [this]() { return t_; }),
    plot::Y(name+ " (x)", [&robot, &fs]() { return fs.wrenchWithoutGravity(robot).force().x(); }, Color::Red, Style::Dashed),
    plot::Y(name+ " (y)", [&robot, &fs]() { return fs.wrenchWithoutGravity(robot).force().y(); }, Color::Green, Style::Dashed),
    plot::Y(name+ " (z)", [&robot, &fs]() { return fs.wrenchWithoutGravity(robot).force().z(); }, Color::Blue, Style::Dashed));
  plots_.push_back(name);
}

void ShowForces::addWrenchVector(const std::string & name, mc_rtc::gui::StateBuilder & gui, const mc_rbdyn::Robot & robot, const mc_rbdyn::ForceSensor & fs)
{
  gui.addElement(category_,
                 Force(name,
                       [&fs]()
                       {
                        return fs.wrench();
                       },
                       [&fs, &robot]()
                       {
                        return fs.X_0_f(robot);
                       }));
}

void ShowForces::addWrenchVector(const std::string & name, const std::string & surface, mc_rtc::gui::StateBuilder & gui, const mc_rbdyn::Robot & robot, const mc_rbdyn::ForceSensor & fs)
{
  gui.addElement(category_,
                 Force(name,
                       [&fs]()
                       {
                        return fs.wrench();
                       },
                       [&fs, &robot, surface]()
                       {
                        return robot.surfacePose(surface);
                       }));
}

void ShowForces::addWrenchWithoutGravityVector(const std::string & name, mc_rtc::gui::StateBuilder & gui, const mc_rbdyn::Robot & robot, const mc_rbdyn::ForceSensor & fs)
{
  gui.addElement(category_,
                 Force(name,
                       [&fs, &robot]()
                       {
                        return fs.wrenchWithoutGravity(robot);
                       },
                       [&fs, &robot]()
                       {
                        return fs.X_0_f(robot);
                       }));
}

void ShowForces::addWrenchWithoutGravityVector(const std::string & name, const std::string & surface, mc_rtc::gui::StateBuilder & gui, const mc_rbdyn::Robot & robot, const mc_rbdyn::ForceSensor & fs)
{
  gui.addElement(category_,
                 Force(name,
                       [&fs, &robot]()
                       {
                        return fs.wrenchWithoutGravity(robot);
                       },
                       [&fs, &robot, surface]()
                       {
                        return robot.surfacePose(surface);
                       }));
}

void ShowForces::start(mc_control::fsm::Controller & ctl)
{
  auto & robot = ctl.robot();

  for(const auto & fs : robot.forceSensors())
  {
    const auto & name = fs.name();
    auto fsCategory = category_;
    fsCategory.push_back(fs.name());
    ctl.gui()->addElement(fsCategory,
                          ElementsStacking::Horizontal,
                          Button("Plot wrench (raw)",
                                 [this, &ctl, &fs]()
                                 {
                                   addWrenchPlot("Wrench " + fs.name(), *ctl.gui(), fs);
                                 }),
                          Button("Stop wrench (raw)",
                                 [this, &ctl, &fs]()
                                 {
                                    ctl.gui()->removePlot("Wrench " + fs.name());
                                 }));


    ctl.gui()->addElement(fsCategory,
                          ElementsStacking::Horizontal,
                          Button("Plot wrench (without gravity)",
                                 [this, &ctl, &fs]()
                                 {
                                   addWrenchWithoutGravityPlot("Wrench without gravity " + fs.name(), *ctl.gui(), ctl.robot(), fs);
                                 }),
                          Button("Stop wrench (without gravity)",
                                 [this, &ctl, &fs]()
                                 {
                                    ctl.gui()->removePlot("Wrench " + fs.name());
                                 }));

    ctl.gui()->addElement(fsCategory,
                          ElementsStacking::Horizontal,
                          Button("Force (raw)",
                                 [this, &ctl, &robot, &fs]()
                                 {
                                  addWrenchVector("Force " + fs.name(), *ctl.gui(), robot, fs);
                                 }),
                          Button("Remove (raw)",
                                 [this, &ctl, &fs]()
                                 {
                                  ctl.gui()->removeElement(category_, "Force " + fs.name());
                                 }));

    ctl.gui()->addElement(fsCategory,
                          ElementsStacking::Horizontal,
                          Button("Force (without gravity)",
                                 [this, &ctl, &robot, &fs]()
                                 {
                                  addWrenchWithoutGravityVector("Force " + fs.name() + "(without gravity)", *ctl.gui(), robot, fs);
                                 }),
                          Button("Remove (without gravity)",
                                 [this, &ctl, &fs]()
                                 {
                                  ctl.gui()->removeElement(category_, "Force " + fs.name() + " (without gravity)");
                                 }));


  }
  output("OK");
}

bool ShowForces::run(mc_control::fsm::Controller & ctl_)
{
  t_ += ctl_.timeStep;
  return true;
}

void ShowForces::teardown(mc_control::fsm::Controller & ctl)
{
  for(const auto & plot : plots_)
  {
    ctl.gui()->removePlot(plot);
  }
  for(const auto & fs : ctl.robot().forceSensors())
  {
    const auto & name = fs.name();
    auto fsCategory = category_;
    fsCategory.push_back(name);
    ctl.gui()->removeCategory(fsCategory);
  }
}

EXPORT_SINGLE_STATE("ShowForces", ShowForces)
