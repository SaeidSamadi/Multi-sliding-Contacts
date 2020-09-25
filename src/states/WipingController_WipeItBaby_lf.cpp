#include "WipingController_WipeItBaby_lf.h"

#include "../WipingController.h"

void WipingController_WipeItBaby_lf::configure(const mc_rtc::Configuration & config)
{
	if(config.has("admittance"))
	{
	  admittance_ = config("admittance");
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
}

void WipingController_WipeItBaby_lf::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);
  ctl.comTask->reset();
  ctl.comTask->weight(2000);
  ctl.comTask->stiffness(60);

 // ctl.lookAtTask->stiffness(5);
 // ctl.lookAtTask->weight(10);
 // ctl.solver().addTask(ctl.lookAtTask);

  ctl.leftFootTask->reset();
  Eigen::Vector6d dimW;
  dimW << 1., 1., 1., 1., 1., 1.;
  sva::MotionVecd stiffnessGain, dampingGain;
  stiffnessGain.angular() << 10, 10, 10;
  stiffnessGain.linear() << 10, 10, 5;
  dampingGain.angular() << 6, 6, 6;
  dampingGain.linear() << 6, 6, 300;
  //stiffnessGain.angular() << 1, 1, 1;
  //stiffnessGain.linear() << 1, 1, 3;
  //dampingGain.angular() << 3, 3, 3;
  //dampingGain.linear() << 3, 3, 3;
  ctl.leftFootTask->setGains(stiffnessGain, dampingGain);
  ctl.leftFootTask->dimWeight(dimW);
  ctl.leftFootTask->admittance(admittance_);
  ctl.leftFootTask->targetCoP(Eigen::Vector2d::Zero());
  //ctl.leftFootTask->targetPose();
  ctl.setTargetFromCoMQP();
  ctl.addLeftFootForceControl();
  
  shift = ctl.realLeftFootPose;
  //ctl.leftFootTask->targetPose(ctl.realLeftFootPose);
  //mc_rtc::log::info("Start.realLeftFoot.translation: {}, \n", ctl.realLeftFootPose.translation());
  //mc_rtc::log::info("Start.realLeftFoot.rotation: {}, \n", ctl.realLeftFootPose.rotation());
  //mc_rtc::log::info("Start.target.translation: {}, \n", ctl.leftFootTask->targetPose().translation());
  //mc_rtc::log::info("Start.target.rotation: {} \n", ctl.leftFootTask->targetPose().rotation());
  ctl.solver().addTask(ctl.comTask);

  ctl.comQP().addToLogger(ctl.logger());

  ctl.logger().addLogEntry("friction_mu_x",
                           [&ctl]()
                           {
                           return ctl.frictionEstimator.mu_x();
                           });
  ctl.logger().addLogEntry("friction_mu_y",
                           [&ctl]()
                           {
                           return ctl.frictionEstimator.mu_y();
                           });
  ctl.logger().addLogEntry("friction_mu",
                           [&ctl]()
                           {
                           return ctl.frictionEstimator.mu();
                           });
}

bool WipingController_WipeItBaby_lf::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);
  //ctl.leftFootTask->targetPose(ctl.realLeftFootPose);
  if(ii == 0){
    mc_rtc::log::info("Run.realLeftFoot.translation: {}, \n", ctl.realLeftFootPose.translation());
    mc_rtc::log::info("Run.realLeftFoot.rotation: {}, \n", ctl.realLeftFootPose.rotation());
    mc_rtc::log::info("Run.target.translation: {}, \n", ctl.leftFootTask->targetPose().translation());
    mc_rtc::log::info("Run.target.rotation: {} \n", ctl.leftFootTask->targetPose().rotation());
  }
  ii += 1;
  if(linearWiping_){
    double linearVelocity = circleRadius_ / wipingDuration_;
    local_x = ctl.timeStep * linearVelocity;
    local_y = 0.0;
  }
  else if(circleWiping_CCW_ || circleWiping_CW_)
  {
    double theta_net = M_PI*3/4;
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

  ctl.setTargetFromCoMQP();

  output("OK");
  return true;
}

void WipingController_WipeItBaby_lf::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);
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
