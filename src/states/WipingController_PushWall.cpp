#include "WipingController_PushWall.h"

#include "../WipingController.h"

void WipingController_PushWall::configure(const mc_rtc::Configuration & config)
{
  if(config.has("useCoMQP"))
  {
    useCoMQP_ = config("useCoMQP");
  }
  if(config.has("maxForce"))
  {
    maxForce_ = config("maxForce");
  }
  if(config.has("duration"))
  {
    duration_ = config("duration");
  }
  if(config.has("maxDuration"))
  {
    maxDuration_ = config("maxDuration");
  }
  if(config.has("admittance"))
  {
    admittance_ = config("admittance");
  }
  if(config.has("initFromCoMQP"))
  {
    initFromCoMQP_ = config("initFromCoMQP");
  }
  if(config.has("forceThreshold"))
  {
    forceThreshold_ = config("forceThreshold");
  }
  if(config.has("comStiffness"))
  {
    comStiffness_ = config("comStiffness");
  }
  stiffness_ << 1,1,1,1,1,1;
  damping_ << 300,300,300,300,300,300;
  if(config.has("stiffness"))
  {
    stiffness_ = config("stiffness");
  }
  if(config.has("damping"))
  {
    damping_ = config("damping");
  }
}

void WipingController_PushWall::start(mc_control::fsm::Controller & ctl_)
{
  LOG_INFO("Starting PushWall with:");
  LOG_INFO("admittance : " << admittance_.transpose());
  LOG_INFO("stiffness : " << stiffness_.transpose());
  LOG_INFO("damping : " << damping_.transpose());

  auto & ctl = static_cast<WipingController &>(ctl_);

  ctl.comTask->reset();
  ctl.comTask->weight(2000);
  ctl.comTask->com(ctl.robot().com());

  ctl.lookAtTask->stiffness(5);
  ctl.lookAtTask->weight(10);
  ctl.solver().addTask(ctl.lookAtTask);

  ctl.admittanceTask->reset();
  ctl.admittanceTask->dimWeight(Eigen::Vector6d::Ones());
  ctl.admittanceTask->admittance(admittance_);
  ctl.admittanceTask->targetCoP(Eigen::Vector2d::Zero());
  ctl.admittanceTask->stiffness(sva::MotionVecd(stiffness_));
  ctl.admittanceTask->damping(sva::MotionVecd(damping_));

  double initialForce = 0.;
  if(initFromCoMQP_)
  {
    initialForce = -ctl.comQP().desiredNormalForce();
    forceTarget_ = sva::ForceVecd({0.,0.,0.}, {0.,0.,initialForce});
  }
  else
  {
    forceTarget_ = ctl.admittanceTask->measuredWrench();
     initialForce = forceTarget_.force().z();
    ctl.admittanceTask->targetForce(forceTarget_.force());
  }
LOG_INFO("initial force: " << initialForce);
LOG_INFO("max force: " << maxForce_);

  // int sign = (initialForce < maxForce_) ? -1 : 1;
  forceRate_ = (maxForce_-initialForce)*ctl.timeStep/duration_;
LOG_INFO("force rate: "<< forceRate_);

  if(useCoMQP_)
  {
    ctl.comQP().desiredNormalForce(-forceTarget_.force().z());
    ctl.computeCoMQP();
    ctl.setTargetFromCoMQP();
    ctl.comTask->stiffness(comStiffness_);
  }
  else
  {
    ctl.admittanceTask->targetForce(forceTarget_.force());
    ctl.comTask->stiffness(comStiffness_);
  }

  ctl.solver().addTask(ctl.comTask);
  ctl.addHandForceControl();
  ctl.addLeftFootForceControl();
  ctl.comQP().addToLogger(ctl.logger());
}

bool WipingController_PushWall::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);

  if(useCoMQP_)
  {
    ctl.comQP().desiredNormalForce(-forceTarget_.force().z());
    ctl.setTargetFromCoMQP();
  }
  else
  {
    ctl.admittanceTask->targetForce(forceTarget_.force());
  }


  if(currTime_ >= maxDuration_ ||
     (currTime_ >= duration_ && std::fabs(ctl.admittanceTask->measuredWrench().force().z() - ctl.admittanceTask->targetWrench().force().z()) < forceThreshold_))
  {
    output("OK");
    return true;
  }

  if(currTime_ < duration_)
  {
    forceTarget_.force().z() += forceRate_;
  }

  currTime_ += ctl.timeStep;
  return false;
}

void WipingController_PushWall::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);
  ctl.removeHandForceControl();
  ctl.removeLeftFootForceControl();
  ctl.solver().removeTask(ctl.comTask);
  ctl.comQP().removeFromLogger(ctl.logger());
  ctl.solver().removeTask(ctl.lookAtTask);
}

EXPORT_SINGLE_STATE("WipingController_PushWall", WipingController_PushWall)
