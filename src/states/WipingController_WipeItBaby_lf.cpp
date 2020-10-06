#include "WipingController_WipeItBaby_lf.h"

#include "../WipingController.h"

void WipingController_WipeItBaby_lf::configure(const mc_rtc::Configuration & config)
{
  if(config.has("maxForce_rh"))
  {
    maxForce_rh_ = config("maxForce_rh");
  }
  if(config.has("maxForce_lh"))
  {
    maxForce_lh_ = config("maxForce_lh");
  }
  if(config.has("maxForce_lf"))
  {
    maxForce_lf_ = config("maxForce_lf");
  }
  if(config.has("admittance_rh"))
  {
    admittance_rh_ = config("admittance_rh");
  }
  if(config.has("admittance_lh"))
  {
    admittance_lh_ = config("admittance_lh");
  }
  if(config.has("admittance_lf"))
  {
    admittance_lf_ = config("admittance_lf");
  }
  if(config.has("feetForceControl"))
  {
    feetForceControl_ = config("feetForceControl");
  }
  if(config.has("linearWiping"))
  {
    linearWiping_ = config("linearWiping");
  }
  if(config.has("circleWiping_CCW"))
  {
    circleWiping_CCW_ = config("circleWiping_CCW");
  }
  if(config.has("circleWiping_CW"))
  {
    circleWiping_CW_ = config("circleWiping_CW");
  }
  if(config.has("circleRadius"))
  {
    circleRadius_ = config("circleRadius");
  }
  if(config.has("wipingDuration"))
  {
    wipingDuration_ = config("wipingDuration");
  }
  if(config.has("comStiffness"))
  {
    comStiffness_ = config("comStiffness");
  }
  if(config.has("stiffness_rh"))
  {
    stiffness_rh_ = config("stiffness_rh");
  }
  if(config.has("stiffness_lh"))
  {
    stiffness_lh_ = config("stiffness_lh");
  }
  if(config.has("stiffness_lf"))
  {
    stiffness_lf_ = config("stiffness_lf");
  }
  if(config.has("damping_rh"))
  {
    damping_rh_ = config("damping_rh");
  }
  if(config.has("damping_lh"))
  {
    damping_lh_ = config("damping_lh");
  }
  if(config.has("damping_lf"))
  {
    damping_lf_ = config("damping_lf");
  }
}

void WipingController_WipeItBaby_lf::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);
  ctl.comTask->reset();
  ctl.comTask->weight(2000);
  ctl.comTask->com(ctl.robot().com()); //XXX Check
  ctl.comTask->stiffness(60);

 // ctl.lookAtTask->stiffness(5);
 // ctl.lookAtTask->weight(10);
 // ctl.solver().addTask(ctl.lookAtTask);
 
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

  ctl.leftFootTask->reset();
  Eigen::Vector6d dimW;
  dimW << 1., 1., 1., 1., 1., 1.;
  //sva::MotionVecd stiffnessGain, dampingGain;
  //stiffnessGain.angular() << 10, 10, 10;
  //stiffnessGain.linear() << 10, 10, 5;
  //dampingGain.angular() << 6, 6, 6;
  //dampingGain.linear() << 6, 6, 300;
  //stiffnessGain.angular() << 1, 1, 1;
  //stiffnessGain.linear() << 1, 1, 3;
  //dampingGain.angular() << 3, 3, 3;
  //dampingGain.linear() << 3, 3, 3;
  //ctl.leftFootTask->setGains(stiffnessGain, dampingGain);
  ctl.leftFootTask->dimWeight(dimW);
  ctl.leftFootTask->admittance(admittance_lf_);
  ctl.leftFootTask->targetCoP(Eigen::Vector2d::Zero());
  ctl.leftFootTask->stiffness(sva::MotionVecd(stiffness_lf_));
  ctl.leftFootTask->damping(sva::MotionVecd(damping_lf_));
  //ctl.leftFootTask->targetPose();
  ctl.setTargetFromCoMQP();
  ctl.addRightHandForceControl();
  ctl.addLeftHandForceControl();
  ctl.addLeftFootForceControl();

  ctl.comQP().rh_desiredNormalForce(-maxForce_rh_);
  ctl.comQP().lh_desiredNormalForce(-maxForce_lh_);
  ctl.comQP().lf_desiredNormalForce(maxForce_lf_);
  forceTarget_rh_ = sva::ForceVecd({0.,0.,0.}, {0.,0.,maxForce_rh_});
  forceTarget_lh_ = sva::ForceVecd({0.,0.,0.}, {0.,0.,maxForce_lh_});
  forceTarget_lf_ = sva::ForceVecd({0.,0.,0.}, {0.,0.,maxForce_lf_});
  ctl.rightHandTask->targetForce(forceTarget_rh_.force());
  ctl.leftHandTask->targetForce(forceTarget_lh_.force());
  ctl.leftFootTask->targetForce(forceTarget_lf_.force());
  shift = ctl.realLeftFootPose;
  //ctl.leftFootTask->targetPose(ctl.realLeftFootPose);
  //mc_rtc::log::info("Start.realLeftFoot.translation: {}, \n", ctl.realLeftFootPose.translation());
  //mc_rtc::log::info("Start.realLeftFoot.rotation: {}, \n", ctl.realLeftFootPose.rotation());
  //mc_rtc::log::info("Start.target.translation: {}, \n", ctl.leftFootTask->targetPose().translation());
  //mc_rtc::log::info("Start.target.rotation: {} \n", ctl.leftFootTask->targetPose().rotation());
  ctl.solver().addTask(ctl.comTask);

  ctl.comQP().addToLogger(ctl.logger());

  //ctl.logger().addLogEntry("friction_mu_x",
  //                         [&ctl]()
  //                         {
  //                         return ctl.frictionEstimator.mu_x();
  //                         });
  //ctl.logger().addLogEntry("friction_mu_y",
  //                         [&ctl]()
  //                         {
  //                         return ctl.frictionEstimator.mu_y();
  //                         });
  //ctl.logger().addLogEntry("friction_mu",
  //                         [&ctl]()
  //                         {
  //                         return ctl.frictionEstimator.mu();
  //                         });
}

bool WipingController_WipeItBaby_lf::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);
  ctl.comQP().rh_desiredNormalForce(-maxForce_rh_);
  ctl.comQP().lh_desiredNormalForce(-maxForce_lh_);
  ctl.comQP().lf_desiredNormalForce(maxForce_lf_);
  //ctl.leftFootTask->targetPose(ctl.realLeftFootPose);
  //if(ii == 0){
  //  mc_rtc::log::info("Run.realLeftFoot.translation: {}, \n", ctl.realLeftFootPose.translation());
  //  mc_rtc::log::info("Run.realLeftFoot.rotation: {}, \n", ctl.realLeftFootPose.rotation());
  //  mc_rtc::log::info("Run.target.translation: {}, \n", ctl.leftFootTask->targetPose().translation());
  //  mc_rtc::log::info("Run.target.rotation: {} \n", ctl.leftFootTask->targetPose().rotation());
  //}
  //ii += 1;
  if(linearWiping_){
    double linearVelocity = circleRadius_ / wipingDuration_;
    local_x = ctl.timeStep * linearVelocity;
    local_y = 0.0;
  }
  else if(circleWiping_CCW_ || circleWiping_CW_)
  {
    double theta_net = M_PI * 3 / 4;
    double angularVelocity = theta_net / wipingDuration_;
    double delta_theta = angularVelocity * ctl.timeStep;
    if(circleWiping_CCW_){
      local_y_initial = - circleRadius_ * sin(theta);
    }
    else if(circleWiping_CW_){
      local_y_initial = circleRadius_ * sin(theta);
    }

    if(theta <= M_PI/2.0){
      local_x_initial = -sqrt(std::pow(circleRadius_, 2.0) - std::pow(local_y_initial, 2.0)) + circleRadius_;
    }
    else{
      local_x_initial = sqrt(std::pow(circleRadius_, 2.0) - std::pow(local_y_initial, 2.0)) + circleRadius_;
    }
    theta += delta_theta;
    if(circleWiping_CCW_){
      local_y_final = - circleRadius_ * sin(theta);
    }
    else if(circleWiping_CW_){
      local_y_final = circleRadius_ * sin(theta);
    }

    if(theta <= M_PI/2.0){
      local_x_final = -sqrt(std::pow(circleRadius_, 2.0) - std::pow(local_y_final, 2.0)) + circleRadius_;
    }
    else{
      local_x_final = sqrt(std::pow(circleRadius_, 2.0) - std::pow(local_y_final, 2.0)) + circleRadius_;
    }
    local_x = local_x_final - local_x_initial;
    local_y = local_y_final - local_y_initial;
  }

  delta_line << local_x, local_y, 0.0; 
  delta_lineW = ctl.slopePosInvW * delta_line;
  wipingTime += ctl.timeStep;

  sva::PTransformd poseOutput = ctl.leftFootTask->targetPose();
  auto rotationPose = poseOutput.rotation();
  auto translationPose = poseOutput.translation();
  
  
  sva::PTransformd target;
  target.rotation() = shift.rotation();
  //target.rotation() = rotationPose;
  //target.rotation() = ctl.realLeftFootPose.rotation().inverse(); //OR .inverse()


  //mc_rtc::log::info("real.leftFoot: \n {}", ctl.leftFootPose);
  //mc_rtc::log::info("real.leftFootRot: \n {}", ctl.leftFootRot);

  if(wipingTime <= wipingDuration_){
    target.translation() = shift.translation();
    shift.translation() += delta_lineW;
    //target.translation() = translationPose + delta_lineW;
    //target.translation() = ctl.realLeftFootPose.translation() + delta_lineW;
  }
  else{
    target.translation() = shift.translation();
    
    //target.translation() = ctl.realLeftFootPose.translation();
  }
  //mc_rtc::log::info("delta_line: \n {}, \n delta_lineW: \n {}", delta_line, delta_lineW);
  //mc_rtc::log::info("target.translation: \n {}", target.translation());
  //mc_rtc::log::info("target.rotation: \n {},", target.rotation());
  ctl.leftFootTask->targetPose(target);
  ctl.comQP().updateLFPose(target);

  ctl.setTargetFromCoMQP();

  output("OK");
  return true;
}

void WipingController_WipeItBaby_lf::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);
  ctl.removeRightHandForceControl();
  ctl.removeLeftHandForceControl();
  ctl.removeLeftFootForceControl();
  ctl.solver().removeTask(ctl.comTask);
  //ctl.solver().removeTask(ctl.lookAtTask);
  //ctl.lookAtTask->stiffness(1);
  //ctl.lookAtTask->weight(1);

  ctl.comQP().removeFromLogger(ctl.logger());

  ctl.logger().removeLogEntry("friction_mu_x");
  ctl.logger().removeLogEntry("friction_mu_y");
  ctl.logger().removeLogEntry("friction_mu");

}

EXPORT_SINGLE_STATE("WipingController_WipeItBaby_lf", WipingController_WipeItBaby_lf)
