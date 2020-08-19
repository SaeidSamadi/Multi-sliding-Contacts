#include "WipingController_PushWall.h"

#include "../WipingController.h"

void WipingController_PushWall::configure(const mc_rtc::Configuration & config)
{
  if(config.has("useCoMQP"))
  {
    useCoMQP_ = config("useCoMQP");
  }
  if(config.has("maxForce_rh"))
  {
    maxForce_rh_ = config("maxForce_rh");
  }
  if(config.has("maxForce_lh"))
  {
    maxForce_lh_ = config("maxForce_lh");
  }
  if(config.has("duration"))
  {
    duration_ = config("duration");
  }
  if(config.has("maxDuration"))
  {
    maxDuration_ = config("maxDuration");
  }
  if(config.has("admittance_rh"))
  {
    admittance_rh_ = config("admittance_rh");
  }
  if(config.has("admittance_lh"))
  {
    admittance_lh_ = config("admittance_lh");
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
  stiffness_rh_ << 1,1,1,1,1,1;
  damping_rh_ << 300,300,300,300,300,300;
  stiffness_lh_ << 1,1,1,1,1,1;
  damping_lh_ << 300,300,300,300,300,300;
  if(config.has("stiffness_rh"))
  {
    stiffness_rh_ = config("stiffness_rh");
  }
  if(config.has("stiffness_lh"))
  {
    stiffness_lh_ = config("stiffness_lh");
  }
  if(config.has("damping_rh"))
  {
    damping_rh_ = config("damping_rh");
  }
  if(config.has("damping_lh"))
  {
    damping_lh_ = config("damping_lh");
  }
}

void WipingController_PushWall::start(mc_control::fsm::Controller & ctl_)
{
  mc_rtc::log::info("Starting PushWall with:");
  mc_rtc::log::info("admittance_rh : {}", admittance_rh_.transpose());
  mc_rtc::log::info("admittance_lh : {}", admittance_lh_.transpose());
  mc_rtc::log::info("stiffness_rh : {}",stiffness_rh_.transpose());
  mc_rtc::log::info("stiffness_lh : {}",stiffness_lh_.transpose());
  mc_rtc::log::info("damping_rh : {}",damping_rh_.transpose());
  mc_rtc::log::info("damping_lh : {}",damping_lh_.transpose());

  auto & ctl = static_cast<WipingController &>(ctl_);

  ctl.comTask->reset();
  ctl.comTask->weight(2000);
  ctl.comTask->com(ctl.robot().com());

  //ctl.lookAtTask->stiffness(5);
  //ctl.lookAtTask->weight(10);
  //ctl.solver().addTask(ctl.lookAtTask);

  ctl.rightHandTask->reset();
  ctl.rightHandTask->dimWeight(Eigen::Vector6d::Ones());
  ctl.rightHandTask->admittance(admittance_rh_);
  ctl.rightHandTask->targetCoP(Eigen::Vector2d::Zero());
  ctl.rightHandTask->stiffness(sva::MotionVecd(stiffness_rh_));
  ctl.rightHandTask->damping(sva::MotionVecd(damping_rh_));

  ctl.leftHandTask->reset();
  ctl.leftHandTask->dimWeight(Eigen::Vector6d::Ones());
  ctl.leftHandTask->admittance(admittance_lh_);
  ctl.leftHandTask->targetCoP(Eigen::Vector2d::Zero());
  ctl.leftHandTask->stiffness(sva::MotionVecd(stiffness_lh_));
  ctl.leftHandTask->damping(sva::MotionVecd(damping_lh_));

  double initialForce_rh = 0.;
  double initialForce_lh = 0.;
  if(initFromCoMQP_)
  {
    initialForce_rh = -ctl.comQP().rh_desiredNormalForce();
    forceTarget_rh_ = sva::ForceVecd({0.,0.,0.}, {0.,0.,initialForce_rh});
    initialForce_lh = -ctl.comQP().lh_desiredNormalForce();
    forceTarget_lh_ = sva::ForceVecd({0.,0.,0.}, {0.,0.,initialForce_lh});
  }
  else
  {
    forceTarget_rh_ = ctl.rightHandTask->measuredWrench();
    initialForce_rh = forceTarget_rh_.force().z();
    ctl.rightHandTask->targetForce(forceTarget_rh_.force());
    forceTarget_lh_ = ctl.leftHandTask->measuredWrench();
    initialForce_lh = forceTarget_lh_.force().z();
    ctl.leftHandTask->targetForce(forceTarget_lh_.force());
  }
  mc_rtc::log::info("right hand initial force: {}", initialForce_rh);
  mc_rtc::log::info("right hand max force: {}", maxForce_rh_);

  forceRate_rh_ = (maxForce_rh_ - initialForce_rh) * ctl.timeStep/duration_;
  forceRate_lh_ = (maxForce_lh_ - initialForce_lh) * ctl.timeStep/duration_;
  mc_rtc::log::info("right hand force rate: {}", forceRate_rh_);
  mc_rtc::log::info("left hand force rate: {}", forceRate_lh_);

  if(useCoMQP_)
  {
    ctl.comQP().rh_desiredNormalForce(-forceTarget_rh_.force().z());
    ctl.comQP().lh_desiredNormalForce(-forceTarget_lh_.force().z());
    ctl.computeCoMQP();
    ctl.setTargetFromCoMQP();
    ctl.comTask->stiffness(comStiffness_);
  }
  else
  {
    ctl.rightHandTask->targetForce(forceTarget_rh_.force());
    ctl.leftHandTask->targetForce(forceTarget_lh_.force());
    ctl.comTask->stiffness(comStiffness_);
  }

  ctl.solver().addTask(ctl.comTask);
  ctl.addRightHandForceControl();
  ctl.addLeftHandForceControl();
  //ctl.addLeftFootForceControl();
  ctl.addFootForceControl();
  ctl.comQP().addToLogger(ctl.logger());
}

bool WipingController_PushWall::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);

  if(useCoMQP_)
  {
    ctl.comQP().rh_desiredNormalForce(-forceTarget_rh_.force().z());
    ctl.comQP().lh_desiredNormalForce(-forceTarget_lh_.force().z());
    ctl.setTargetFromCoMQP();
  }
  else
  {
    ctl.rightHandTask->targetForce(forceTarget_rh_.force());
    ctl.leftHandTask->targetForce(forceTarget_lh_.force());
  }

  if(currTime_ >= maxDuration_ ||
     (currTime_ >= duration_ && std::fabs(ctl.rightHandTask->measuredWrench().force().z() - ctl.rightHandTask->targetWrench().force().z()) < forceThreshold_ &&
                                std::fabs(ctl.leftHandTask->measuredWrench().force().z() - ctl.leftHandTask->targetWrench().force().z()) < forceThreshold_))
  {
    output("OK");
    return true;
  }

  if(currTime_ < duration_)
  {
    forceTarget_rh_.force().z() += forceRate_rh_;
    forceTarget_lh_.force().z() += forceRate_lh_;
  }

  currTime_ += ctl.timeStep;
  return false;
}

void WipingController_PushWall::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);
  ctl.removeRightHandForceControl();
  ctl.removeLeftHandForceControl();
  //ctl.removeLeftFootForceControl();
  ctl.removeFootForceControl();
  ctl.solver().removeTask(ctl.comTask);
  ctl.comQP().removeFromLogger(ctl.logger());
  //ctl.solver().removeTask(ctl.lookAtTask);
}

EXPORT_SINGLE_STATE("WipingController_PushWall", WipingController_PushWall)
