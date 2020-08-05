#pragma once

#include <mc_control/fsm/State.h>
#include <SpaceVecAlg/SpaceVecAlg>

struct WipingController_PushWall_lf : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;
 private:
  bool useCoMQP_ = true;
  double maxForce_ = 150;
  double duration_ = 3.0;
  double maxDuration_ = 5.;
  double forceThreshold_ = 5.;
  sva::ForceVecd forceTarget_;
  double forceRate_;
  Eigen::Vector6d admittance_;
  Eigen::Vector6d stiffness_;
  Eigen::Vector6d damping_;

  double comStiffness_ = 20;

  double currTime_ = 0.;

  bool initFromCoMQP_ = false;
};
