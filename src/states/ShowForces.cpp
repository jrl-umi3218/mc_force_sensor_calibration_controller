#include "ShowForces.h"
#include <mc_control/fsm/Controller.h>
#include <mc_rbdyn/Robot.h>
#include <mc_rtc/gui.h>

using namespace mc_rtc::gui;
using Style = mc_rtc::gui::plot::Style;

void ShowForces::configure(const mc_rtc::Configuration & config)
{
  config("category", category_);
  config("forceScale", forceScale_);
}

void ShowForces::addWrenchPlot(const std::string & name,
                               mc_rtc::gui::StateBuilder & gui,
                               const mc_rbdyn::ForceSensor & fs)
{
  gui.addPlot(name, plot::X("t", [this]() { return t_; }),
              plot::Y(name + " (x)", [&fs]() { return fs.wrench().force().x(); }, Color::Red, Style::Dashed),
              plot::Y(name + " (y)", [&fs]() { return fs.wrench().force().y(); }, Color::Green, Style::Dashed),
              plot::Y(name + " (z)", [&fs]() { return fs.wrench().force().z(); }, Color::Blue, Style::Dashed));
  plots_.push_back(name);
}

void ShowForces::addWrenchWithoutGravityPlot(const std::string & name,
                                             mc_rtc::gui::StateBuilder & gui,
                                             const mc_rbdyn::Robot & robot,
                                             const mc_rbdyn::ForceSensor & fs)
{
  gui.addPlot(name, plot::X("t", [this]() { return t_; }),
              plot::Y(name + " (x)", [&robot, &fs]() { return fs.wrenchWithoutGravity(robot).force().x(); }, Color::Red,
                      Style::Dashed),
              plot::Y(name + " (y)", [&robot, &fs]() { return fs.wrenchWithoutGravity(robot).force().y(); },
                      Color::Green, Style::Dashed),
              plot::Y(name + " (z)", [&robot, &fs]() { return fs.wrenchWithoutGravity(robot).force().z(); },
                      Color::Blue, Style::Dashed));
  plots_.push_back(name);
}

void ShowForces::addWrenchWithoutGravityPlot(const std::string & name,
                                             const std::string & surface,
                                             mc_rtc::gui::StateBuilder & gui,
                                             const mc_rbdyn::Robot & robot,
                                             const mc_rbdyn::ForceSensor & fs)
{
  gui.addPlot(name, plot::X("t", [this]() { return t_; }),
              plot::Y(name + " (x)", [&robot, &fs, surface]() { return robot.surfaceWrench(surface).force().x(); },
                      Color::Red, Style::Dashed),
              plot::Y(name + " (y)", [&robot, &fs, surface]() { return robot.surfaceWrench(surface).force().y(); },
                      Color::Green, Style::Dashed),
              plot::Y(name + " (z)", [&robot, &fs, surface]() { return robot.surfaceWrench(surface).force().z(); },
                      Color::Blue, Style::Dashed));
  plots_.push_back(name);
}

void ShowForces::addWrenchVector(const std::string & name,
                                 mc_rtc::gui::StateBuilder & gui,
                                 const mc_rbdyn::Robot & robot,
                                 const mc_rbdyn::ForceSensor & fs)
{
  gui.addElement(category_, Force(name, forceConfig_, [&fs]() { return fs.wrench(); },
                                  [&fs, &robot]() { return fs.X_0_f(robot); }));
}

void ShowForces::addWrenchWithoutGravityVector(const std::string & name,
                                               mc_rtc::gui::StateBuilder & gui,
                                               const mc_rbdyn::Robot & robot,
                                               const mc_rbdyn::ForceSensor & fs)
{
  gui.addElement(category_, Force(name, forceConfig_, [&fs, &robot]() { return fs.wrenchWithoutGravity(robot); },
                                  [&fs, &robot]() { return fs.X_0_f(robot); }));
}

void ShowForces::addWrenchWithoutGravityVector(const std::string & name,
                                               const std::string & surface,
                                               mc_rtc::gui::StateBuilder & gui,
                                               const mc_rbdyn::Robot & robot,
                                               const mc_rbdyn::ForceSensor & fs)
{
  gui.addElement(category_, Force(name, forceConfig_, [&fs, &robot, surface]() { return robot.surfaceWrench(surface); },
                                  [&fs, &robot, surface]() { return robot.surfacePose(surface); }));
}

void ShowForces::start(mc_control::fsm::Controller & ctl)
{
  auto & robot = ctl.robot();

  ctl.gui()->addElement(category_, Button("Stop", [this]() { stop_ = true; }));

  forceConfig_.force_scale *= forceScale_;
  for(const auto & fs : robot.forceSensors())
  {
    const auto & name = fs.name();
    auto fsCategory = category_;
    fsCategory.push_back(fs.name());
    ctl.gui()->addElement(
        fsCategory, ElementsStacking::Horizontal,
        Button("Plot wrench (raw)", [this, &ctl, &fs]() { addWrenchPlot("Wrench " + fs.name(), *ctl.gui(), fs); }),
        Button("Stop wrench (raw)", [this, &ctl, &fs]() { ctl.gui()->removePlot("Wrench " + fs.name()); }));

    ctl.gui()->addElement(
        fsCategory, ElementsStacking::Horizontal,
        Button("Plot wrench (without gravity)",
               [this, &ctl, &fs]() {
                 addWrenchWithoutGravityPlot("Wrench without gravity " + fs.name(), *ctl.gui(), ctl.robot(), fs);
               }),
        Button("Stop wrench (without gravity)", [this, &ctl, &fs]() { ctl.gui()->removePlot("Wrench " + fs.name()); }));

    ctl.gui()->addElement(
        fsCategory, ElementsStacking::Horizontal,
        Button("Force (raw)",
               [this, &ctl, &robot, &fs]() { addWrenchVector("Force " + fs.name(), *ctl.gui(), robot, fs); }),
        Button("Remove (raw)", [this, &ctl, &fs]() { ctl.gui()->removeElement(category_, "Force " + fs.name()); }));

    ctl.gui()->addElement(fsCategory, ElementsStacking::Horizontal,
                          Button("Force (without gravity)",
                                 [this, &ctl, &robot, &fs]() {
                                   addWrenchWithoutGravityVector("Force " + fs.name() + " (without gravity)",
                                                                 *ctl.gui(), robot, fs);
                                 }),
                          Button("Remove (without gravity)", [this, &ctl, &fs]() {
                            ctl.gui()->removeElement(category_, "Force " + fs.name() + " (without gravity)");
                          }));

    std::vector<std::string> surfaces;
    for(const auto & surface : robot.surfaces())
    {
      try
      {
        const auto & body = surface.second->bodyName();
        const auto & fs =
            robot.bodyHasForceSensor(body) ? robot.bodyForceSensor(body) : robot.indirectBodyForceSensor(body);
        if(fs.name() == name)
        {
          surfaces.push_back(surface.first);
        }
      }
      catch(...)
      {
      }
    }
    if(surfaces.size())
    {
      surfaces_[name] = surfaces.front();
      ctl.gui()->addElement(fsCategory,
                            mc_rtc::gui::ComboInput("Surface", surfaces, [this, name]() { return surfaces_[name]; },
                                                    [this, name](const std::string & surface) {
                                                      mc_rtc::log::info("[ShowForces] Surface {} selected", surface);
                                                      surfaces_[name] = surface;
                                                    }));
      // mc  _rtc::gui::FormDataComboInput{"R0 surface", false, {"surfaces", "$R0"}},

      ctl.gui()->addElement(fsCategory, ElementsStacking::Horizontal,
                            Button("Plot surface wrench (without gravity)",
                                   [this, &ctl, &fs]() {
                                     addWrenchWithoutGravityPlot("Wrench without gravity " + fs.name(),
                                                                 surfaces_[fs.name()], *ctl.gui(), ctl.robot(), fs);
                                   }),
                            Button("Stop surface wrench (without gravity)",
                                   [this, &ctl, &fs]() { ctl.gui()->removePlot("Wrench " + fs.name()); }));
      ctl.gui()->addElement(fsCategory, ElementsStacking::Horizontal,
                            Button("Surface Force (without gravity)",
                                   [this, &ctl, &robot, &fs]() {
                                     addWrenchWithoutGravityVector("Surface Force " + fs.name() + " (without gravity)",
                                                                   surfaces_[fs.name()], *ctl.gui(), robot, fs);
                                   }),
                            Button("Remove Surface Force (without gravity)", [this, &ctl, &fs]() {
                              ctl.gui()->removeElement(category_, "Surface Force " + fs.name() + " (without gravity)");
                            }));
    }
  }
  output("OK");
}

bool ShowForces::run(mc_control::fsm::Controller & ctl_)
{
  t_ += ctl_.timeStep;
  return stop_;
}

void ShowForces::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeElement(category_, "Stop");
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
