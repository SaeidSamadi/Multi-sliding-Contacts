#pragma once

#include <mc_control/fsm/State.h>

struct WipingController_WipeItBaby_lh : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void addTunningGUI(mc_control::fsm::Controller & ctl);
  void removeTunningGUI(mc_control::fsm::Controller & ctl);

  void resetTrajectory();
  void startResumeTrajectory();
  void pauseTrajectory();
  void updateTrajectory(double timeStep);

  void addTunedGainsToConf( mc_rtc::Configuration & config);
  void loadTunedGainsFromConf( mc_rtc::Configuration & config);
  void saveTunedGains();
  
 protected:
  Eigen::Vector6d admittance_ = Eigen::Vector6d::Zero();
  Eigen::Vector6d stiffness_ = Eigen::Vector6d::Zero();
  Eigen::Vector6d damping_ = Eigen::Vector6d::Zero();
  
  bool feetForceControl_ = true;
  bool linearWiping_ = false;
  bool circleWiping_CCW_ = false;
  bool circleWiping_CW_ = false;
  double amplitude_ = 0.0;
  double orientation_ = -M_PI;
  double wipingDuration_ = 0.0;
  double local_x, local_y;
  double local_x_initial, local_y_initial;
  double local_x_prev, local_y_prev;
  double wipingTime_ = 0;
  bool Wiping_= true;
  double theta = 0.0;
  Eigen::Vector3d delta_line, delta_lineW;

  bool tune_ = false;
  std::string categoryName_ = "Left Hand Tunning";

  double t_;
};
