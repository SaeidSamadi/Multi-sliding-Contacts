#pragma once

#include <mc_control/fsm/State.h>
#include <math.h>

struct WipingController_WipeItBaby_rh : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

 protected:
	Eigen::Vector6d admittance_ = Eigen::Vector6d::Zero();
  double xx = 0;
  bool linearWiping_ = false;
  bool circleWiping_CCW_ = false;
  bool circleWiping_CW_ = false;
  double circleRadius_ = 0.0; 
  double wipingDuration_ = 0.0;
  double local_x, local_y;
  double local_x_initial, local_y_initial;
  double local_x_final, local_y_final;
  bool feetForceControl_ = true;
  double wipingTime = 0;
  double theta = 0.0;
  Eigen::Vector3d delta_line, delta_lineW;
};
