#include "WipingController_ConstantAcceleration.h"

#include "../WipingController.h"

void WipingController_ConstantAcceleration::configure(const mc_rtc::Configuration & config)
{
  LOG_INFO("called with config " << config.dump(true));
  if(config.has("refAccel"))
  {
    refAccel_ = config("refAccel", sva::MotionVecd::Zero());
    LOG_WARNING(refAccel_);
    speedThreshold_ = config("evalSpeed", 0.0);
    vel_ = config("initVel", sva::MotionVecd::Zero());
  }
}

void WipingController_ConstantAcceleration::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);
  surfaceTranformTask_.reset(new mc_tasks::SurfaceTransformTask("RightHandPad", ctl.robots(), ctl.robots().robotIndex(), 5., 1000.));
  surfaceTranformTask_->reset();
  surfaceTranformTask_->name("ConstantAccelerationTask");

  Eigen::Vector6d dimW;
  dimW << 0.,0.,0.,1.,0.,0.;
  surfaceTranformTask_->dimWeight(dimW);
  surfaceTranformTask_->setGains(0, 300);
  //vel_.vector() = surfaceTranformTask_->speed();
  //vel_ = ctl.robot().surface("RightHandPad").vel(ctl.robot());
  surfaceTranformTask_->refAccel(refAccel_);
  surfaceTranformTask_->refVelB(vel_);
  ctl.solver().addTask(surfaceTranformTask_);
}

bool WipingController_ConstantAcceleration::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);

  vel_ += ctl.timeStep*refAccel_;
  surfaceTranformTask_->refVelB(vel_.vector());
  surfaceTranformTask_->refAccel(refAccel_);

  output("OK");
  return std::abs(vel_.vector().norm() - speedThreshold_) < 0.001;
}

void WipingController_ConstantAcceleration::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);
  ctl.solver().removeTask(surfaceTranformTask_);
}

EXPORT_SINGLE_STATE("WipingController_ConstantAcceleration", WipingController_ConstantAcceleration)
