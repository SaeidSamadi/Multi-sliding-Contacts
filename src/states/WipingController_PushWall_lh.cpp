#include "WipingController_PushWall_lh.h"

#include "../WipingController.h"

void WipingController_PushWall_lh::configure(const mc_rtc::Configuration & config)
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

void WipingController_PushWall_lh::start(mc_control::fsm::Controller & ctl_)
{
  mc_rtc::log::info("Starting PushWall LH with:");
  mc_rtc::log::info("admittance : {}", admittance_.transpose());
  mc_rtc::log::info("stiffness : {}", stiffness_.transpose());
  mc_rtc::log::info("damping : {}", damping_.transpose());

  auto & ctl = static_cast<WipingController &>(ctl_);

  ctl.comTask->reset();
  ctl.comTask->weight(2000);
  ctl.comTask->com(ctl.robot().com());

  //ctl.lookAtTask->stiffness(5);
  //ctl.lookAtTask->weight(10);
  //ctl.solver().addTask(ctl.lookAtTask);

  ctl.leftHandTask->reset();
  ctl.leftHandTask->dimWeight(Eigen::Vector6d::Ones());
  ctl.leftHandTask->admittance(admittance_);
  ctl.leftHandTask->targetCoP(Eigen::Vector2d::Zero());
  ctl.leftHandTask->stiffness(sva::MotionVecd(stiffness_));
  ctl.leftHandTask->damping(sva::MotionVecd(damping_));

  double initialForce = 0.;
  if(initFromCoMQP_)
  {
    initialForce = -ctl.comQP().lh_desiredNormalForce();
    forceTarget_ = sva::ForceVecd({0.,0.,0.}, {0.,0.,initialForce});
  }
  else
  {
    forceTarget_ = ctl.leftHandTask->measuredWrench();
    initialForce = std::max(forceTarget_.force().z(), 0.00001);
    forceTarget_.force().z() = initialForce;
    ctl.leftHandTask->targetForce(forceTarget_.force());
  }
  mc_rtc::log::info("initial force: {}", initialForce);
  mc_rtc::log::info("max force: {}", maxForce_);

  // int sign = (initialForce < maxForce_) ? -1 : 1;
  forceRate_ = (maxForce_-initialForce)*ctl.timeStep/duration_;
  mc_rtc::log::info("force rate: {}", forceRate_);

  if(useCoMQP_)
  {
    ctl.comQP().lh_desiredNormalForce(-forceTarget_.force().z());
    ctl.computeCoMQP();
    ctl.setTargetFromCoMQP();
    ctl.comTask->stiffness(comStiffness_);
  }
  else
  {
    ctl.leftHandTask->targetForce(forceTarget_.force());
    ctl.comTask->stiffness(comStiffness_);
  }

  ctl.solver().addTask(ctl.comTask);
  ctl.addLeftHandForceControl();
  //ctl.addLeftFootForceControl();
  ctl.addFootForceControl();
  ctl.comQP().addToLogger(ctl.logger());
}

bool WipingController_PushWall_lh::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);

  if(useCoMQP_)
  {
    ctl.comQP().lh_desiredNormalForce(-forceTarget_.force().z());
    ctl.setTargetFromCoMQP();
  }
  else
  {
    ctl.leftHandTask->targetForce(forceTarget_.force());
  }


  if(currTime_ >= maxDuration_ ||
     (currTime_ >= duration_ && std::fabs(ctl.leftHandTask->measuredWrench().force().z() - ctl.leftHandTask->targetWrench().force().z()) < forceThreshold_))
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

void WipingController_PushWall_lh::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);
  ctl.removeLeftHandForceControl();
  //ctl.removeLeftFootForceControl();
  ctl.removeFootForceControl();
  ctl.solver().removeTask(ctl.comTask);
  ctl.comQP().removeFromLogger(ctl.logger());
  //ctl.solver().removeTask(ctl.lookAtTask);
}

EXPORT_SINGLE_STATE("WipingController_PushWall_lh", WipingController_PushWall_lh)
