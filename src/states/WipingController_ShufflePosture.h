#pragma once

#include <mc_control/fsm/State.h>
#include <SpaceVecAlg/SpaceVecAlg>

struct WipingController_ShufflePosture : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;
 private:
  bool useCoMQP_ = true;
  double maxForce_rh_ = -80.0;
  double maxForce_lh_ = -80.0;
  double maxForce_lf_ = 150.0;
  double duration_ = 3.0;
  double maxDuration_ = 5.;
  double forceThreshold_ = 5.;
  sva::ForceVecd forceTarget_rh_, forceTarget_lh_, forceTarget_lf_;
  double forceRate_rh_, forceRate_lh_, forceRate_lf_;
  Eigen::Vector6d admittance_rh_, admittance_lh_, admittance_lf_;
  Eigen::Vector6d stiffness_rh_, stiffness_lh_, stiffness_lf_;
  Eigen::Vector6d damping_rh_, damping_lh_, damping_lf_;

  double comStiffness_ = 20;

  double currTime_ = 0.;

  bool initFromCoMQP_ = false;
};
