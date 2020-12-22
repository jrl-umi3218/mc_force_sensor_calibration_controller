#include "ChooseTransition.h"
#include <mc_control/fsm/Controller.h>
#include <mc_rtc/gui/Button.h>

void ChooseTransition::configure(const mc_rtc::Configuration & config)
{
  config("category", category_);
  config("actions", actions_);
}

void ChooseTransition::start(mc_control::fsm::Controller & ctl)
{
  using namespace mc_rtc::gui;
  for(const auto & action : actions_)
  {
    ctl.gui()->addElement(category_, Button(action.first, [this, action]() {
                            mc_rtc::log::info("[{}] Action {} chosen, triggering output {}", name(), action.first,
                                              action.second);
                            output(action.second);
                          }));
  }
}

bool ChooseTransition::run(mc_control::fsm::Controller &)
{
  if(output().empty())
  {
    return false;
  }
  return true;
}

void ChooseTransition::teardown(mc_control::fsm::Controller & ctl)
{
  for(const auto & action : actions_)
  {
    ctl.gui()->removeElement(category_, action.first);
  }
}

EXPORT_SINGLE_STATE("ChooseTransition", ChooseTransition)
