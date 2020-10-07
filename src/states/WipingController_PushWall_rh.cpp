#include "WipingController_PushWall_rh.h"

#include "../WipingController.h"

void WipingController_PushWall_rh::configure(const mc_rtc::Configuration & config)
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

void WipingController_PushWall_rh::start(mc_control::fsm::Controller & ctl_)
{
  mc_rtc::log::info("Starting PushWal RH with:");
  mc_rtc::log::info("admittance : {}", admittance_.transpose());
  mc_rtc::log::info("stiffness : {}", stiffness_.transpose());
  mc_rtc::log::info("damping : {}", damping_.transpose());

  auto & ctl = static_cast<WipingController &>(ctl_);

  ctl.comTask->reset();
  ctl.comTask->weight(2000);
  ctl.comTask->com(ctl.robot().com());

  ctl.lookAtTask->stiffness(5);
  ctl.lookAtTask->weight(10);
  ctl.solver().addTask(ctl.lookAtTask);

  ctl.rightHandTask->reset();
  ctl.rightHandTask->dimWeight(Eigen::Vector6d::Ones());
  ctl.rightHandTask->admittance(admittance_);
  ctl.rightHandTask->targetCoP(Eigen::Vector2d::Zero());
  ctl.rightHandTask->stiffness(sva::MotionVecd(stiffness_));
  ctl.rightHandTask->damping(sva::MotionVecd(damping_));

  double initialForce = 0.;
  if(initFromCoMQP_)
  {
    initialForce = -ctl.comQP().rh_desiredNormalForce();
    forceTarget_ = sva::ForceVecd({0.,0.,0.}, {0.,0.,initialForce});
  }
  else
  {
    forceTarget_ = ctl.rightHandTask->measuredWrench();
    initialForce = std::max(forceTarget_.force().z(), 0.00001);
    forceTarget_.force().z() = initialForce;
    ctl.rightHandTask->targetForce(forceTarget_.force());
  }
mc_rtc::log::info("initial force: {}", initialForce);
mc_rtc::log::info("max force: {}", maxForce_);

  // int sign = (initialForce < maxForce_) ? -1 : 1;
  forceRate_ = (maxForce_-initialForce)*ctl.timeStep/duration_;
mc_rtc::log::info("force rate: {}", forceRate_);

  if(useCoMQP_)
  {
    ctl.comQP().rh_desiredNormalForce(-forceTarget_.force().z());
    ctl.computeCoMQP();
    ctl.setTargetFromCoMQP();
    ctl.comTask->stiffness(comStiffness_);
  }
  else
  {
    ctl.rightHandTask->targetForce(forceTarget_.force());
    ctl.comTask->stiffness(comStiffness_);
  }

  ctl.solver().addTask(ctl.comTask);
  ctl.addRightHandForceControl();
  //ctl.addLeftFootForceControl();
  ctl.addFootForceControl();
  ctl.comQP().addToLogger(ctl.logger());
}

bool WipingController_PushWall_rh::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);

  if(useCoMQP_)
  {
    ctl.comQP().rh_desiredNormalForce(-forceTarget_.force().z());
    ctl.setTargetFromCoMQP();
  }
  else
  {
    ctl.rightHandTask->targetForce(forceTarget_.force());
  }


  if(currTime_ >= maxDuration_ ||
     (currTime_ >= duration_ && std::fabs(ctl.rightHandTask->measuredWrench().force().z() - ctl.rightHandTask->targetWrench().force().z()) < forceThreshold_))
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

void WipingController_PushWall_rh::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);
  ctl.removeRightHandForceControl();
  //ctl.removeLeftFootForceControl();
  ctl.removeFootForceControl();
  ctl.solver().removeTask(ctl.comTask);
  ctl.comQP().removeFromLogger(ctl.logger());
  ctl.solver().removeTask(ctl.lookAtTask);
}

EXPORT_SINGLE_STATE("WipingController_PushWall_rh", WipingController_PushWall_rh)
