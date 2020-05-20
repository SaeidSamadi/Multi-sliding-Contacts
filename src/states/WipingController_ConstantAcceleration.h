#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/SurfaceTransformTask.h>

struct WipingController_ConstantAcceleration : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;
 private:
  sva::MotionVecd vel_ = sva::MotionVecd::Zero();
  sva::MotionVecd refAccel_ = sva::MotionVecd::Zero();
  double speedThreshold_ = 0.;

  std::shared_ptr<mc_tasks::SurfaceTransformTask> surfaceTranformTask_;
};
