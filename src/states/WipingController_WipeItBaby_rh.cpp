#include "WipingController_WipeItBaby_rh.h"

#include "../WipingController.h"

void WipingController_WipeItBaby_rh::configure(const mc_rtc::Configuration & config)
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

void WipingController_WipeItBaby_rh::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);

  if(feetForceControl_)
  {
    ctl.addFootForceControl();
  }
  else
  {
    ctl.removeFootForceControl();
  }
  ctl.comTask->reset();
  ctl.comTask->weight(2000);
  ctl.comTask->stiffness(60);

  ctl.lookAtTask->stiffness(5);
  ctl.lookAtTask->weight(10);
  ctl.solver().addTask(ctl.lookAtTask);

  ctl.rightHandTask->reset();
  //ctl.rightHandTask->setGains(1, 300);
  //ctl.rightHandTask->stiffness(1.);
  //ctl.rightHandTask->damping(300.);
  Eigen::Vector6d dimW;
  dimW << 1., 1., 1., 1., 1., 1.;
  sva::MotionVecd stiffnessGain, dampingGain;
  stiffnessGain.angular() << 10, 10, 10;
  stiffnessGain.linear() << 10, 10, 5;
  dampingGain.angular() << 6, 6, 6;
  dampingGain.linear() << 6, 6, 300;
  ctl.rightHandTask->setGains(stiffnessGain, dampingGain);
  ctl.rightHandTask->dimWeight(dimW);
  ctl.rightHandTask->admittance(admittance_);
  ctl.rightHandTask->targetCoP(Eigen::Vector2d::Zero());
  ctl.rightHandTask->targetPose();
  ctl.setTargetFromCoMQP();
  ctl.addRightHandForceControl();

  //ctl.setFeetTargetFromCoMQP();
  //ctl.addLeftFootForceControl();
  ctl.solver().addTask(ctl.comTask);

  ctl.comQP().addToLogger(ctl.logger());
  //xx = 10.0;
  //ctl.logger().addLogEntry("EstimatedFriction",
  //                         []()
  //                         { double xxx = 100.0;
  //                         return xx;
  //                         });

  ctl.logger().addLogEntry("friction_mu_Estim_rh",
                           [&ctl]()
                           {
                           return ctl.frictionEstimator_rh.mu_calc();
                           });
  ctl.logger().addLogEntry("friction_mu_filtered_rh",
                           [&ctl]()
                           {
                           return ctl.frictionEstimator_rh.mu_filtered();
                           });
  ctl.logger().addLogEntry("SurfaceWrench_x",
                           [&ctl]()
                           {
                           return ctl.frictionEstimator_rh.forceX();
                           });
  ctl.logger().addLogEntry("SurfaceWrench_y",
                           [&ctl]()
                           {
                           return ctl.frictionEstimator_rh.forceY();
                           });
  ctl.logger().addLogEntry("SurfaceWrench_z",
                           [&ctl]()
                           {
                           return ctl.frictionEstimator_rh.forceZ();
                           });
}

bool WipingController_WipeItBaby_rh::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);
  if(linearWiping_){
    double linearVelocity = circleRadius_ / wipingDuration_;
    local_x = ctl.timeStep * linearVelocity;
    local_y = 0.0;
  }
  else if(circleWiping_CCW_ || circleWiping_CW_)
  {
    double theta_net = 2 * M_PI;
    double angleRate = theta_net / wipingDuration_;
    double delta_theta = angleRate * ctl.timeStep;
    if(circleWiping_CCW_){
      local_y_initial = - circleRadius_ * sin(theta);
    }
    else if(circleWiping_CW_){
      local_y_initial = circleRadius_ * sin(theta);
    }

    if((theta <= M_PI/2.0) || (theta >= 1.5 * M_PI && theta <= 2 * M_PI)){
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

    if((theta <= M_PI/2.0) || (theta >= 1.5 * M_PI && theta <= 2 * M_PI)){
      local_x_final = -sqrt(std::pow(circleRadius_, 2.0) - std::pow(local_y_final, 2.0)) + circleRadius_;
    }
    else{
      local_x_final = sqrt(std::pow(circleRadius_, 2.0) - std::pow(local_y_final, 2.0)) + circleRadius_;
    }
    local_x = local_x_final - local_x_initial;
    local_y = local_y_final - local_y_initial;
  }

  delta_line << local_x, local_y, 0.0; //This should generate straight line wiping on 45deg tilted_board
  delta_lineW = ctl.tiltedboardPosInvW * delta_line;
  wipingTime += ctl.timeStep;
  sva::PTransformd poseOutput = ctl.rightHandTask->targetPose();
  auto rotationPose = poseOutput.rotation();
  auto translationPose = poseOutput.translation();
  sva::PTransformd target;
  target.rotation() = rotationPose;
  if(wipingTime <= wipingDuration_){
    target.translation() = translationPose + delta_lineW;
  }
  else{
    target.translation() = translationPose;
  }
  ctl.rightHandTask->targetPose(target);

  ctl.comQP().updateRHPose(target);
  ctl.frictionEstimator_rh.update(ctl.robot());
  //double EstimatedFriction = ctl.frictionEstimator_rh.mu_calc();

  //mc_rtc::log::info("EstimatedFriction: {}", EstimatedFriction);
  ctl.setTargetFromCoMQP();
  //ctl.setFeetTargetFromCoMQP();
  // handForceFilter_.add(ctl.robot().forceSensor("RightHandForceSensor").worldWrench(ctl.robot()));

  output("OK");
  return true;
}

void WipingController_WipeItBaby_rh::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);
  ctl.removeRightHandForceControl();
  //ctl.removeLeftFootForceControl();
  ctl.removeFootForceControl();
  ctl.solver().removeTask(ctl.comTask);
  ctl.solver().removeTask(ctl.lookAtTask);

  ctl.lookAtTask->stiffness(1);
  ctl.lookAtTask->weight(1);

  ctl.comQP().removeFromLogger(ctl.logger());

  ctl.logger().removeLogEntry("friction_mu_x_Estim");
  //ctl.logger().removeLogEntry("friction_mu_y");
  //ctl.logger().removeLogEntry("friction_mu");

}

EXPORT_SINGLE_STATE("WipingController_WipeItBaby_rh", WipingController_WipeItBaby_rh)
