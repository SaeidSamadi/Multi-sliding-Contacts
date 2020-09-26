#pragma once

#include <mc_control/fsm/State.h>

struct WipingController_WipeItBaby_lf : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

 protected:
  Eigen::Vector6d admittance_rh_, admittance_lh_, admittance_lf_;
  Eigen::Vector6d stiffness_rh_, stiffness_lh_, stiffness_lf_;
  Eigen::Vector6d damping_rh_, damping_lh_, damping_lf_;
  double maxForce_rh_ = 20.0;
  double maxForce_lh_ = 20.0;
  double maxForce_lf_ = 30.0;
  double comStiffness_ = 20;
  bool feetForceControl_ = false;
  bool linearWiping_ = false;
  bool circleWiping_CCW_ = false;
  bool circleWiping_CW_ = false;
  double circleRadius_ = 0.0; 
  double wipingDuration_ = 0.0;
  double local_x, local_y;
  double local_x_initial, local_y_initial;
  double local_x_final, local_y_final;
  double wipingTime = 0;
  double theta = 0.0;
  sva::ForceVecd forceTarget_rh_, forceTarget_lh_, forceTarget_lf_;
  int ii = 0;
  sva::PTransformd shift;
  Eigen::Vector3d delta_line, delta_lineW;
};
