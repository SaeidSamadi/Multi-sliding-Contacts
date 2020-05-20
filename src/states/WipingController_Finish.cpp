#include "WipingController_Finish.h"

#include "../WipingController.h"

void WipingController_Finish::configure(const mc_rtc::Configuration & config)
{
}

void WipingController_Finish::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);
  ctl.removeLeftFootForceControl();
}

bool WipingController_Finish::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);
  output("OK");
  return true;
}

void WipingController_Finish::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);
}

EXPORT_SINGLE_STATE("WipingController_Finish", WipingController_Finish)
