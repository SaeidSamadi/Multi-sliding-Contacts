#pragma once

#include <mc_control/fsm/State.h>

struct WipingController_WipeItBaby_lh : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

 protected:
	Eigen::Vector6d admittance_ = Eigen::Vector6d::Zero();
  bool feetForceControl_ = true;
};
