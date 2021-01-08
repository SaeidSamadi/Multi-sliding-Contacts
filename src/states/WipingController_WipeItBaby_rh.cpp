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
      amplitude_ = config("circleRadius");
    }
  if(config.has("wipingDuration"))
    {
      wipingDuration_ = config("wipingDuration");
    }

  if (config.has("tune")) tune_ = config("tune");
}

void WipingController_WipeItBaby_rh::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);

  t_ = 0.0;
  if (tune_){
    mc_rtc::log::warning("Tunning Mode Activated!");
    addTunningGUI(ctl);
  }
  
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
  admittanceZ_ = admittance_(5);
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

  t_ += ctl.timeStep;
  updateTrajectory(ctl.timeStep);

  delta_line << local_x - local_x_prev,
                local_y - local_y_prev,
                0.0;
  
  //This should generate straight line wiping on 45deg tilted_board
  delta_lineW = ctl.tiltedboardPosInvW * delta_line;
  sva::PTransformd poseOutput = ctl.rightHandTask->targetPose();
  auto rotationPose = poseOutput.rotation();
  auto translationPose = poseOutput.translation();
  sva::PTransformd target;
  sva::MotionVecd targetVelocity = sva::MotionVecd::Zero();
  target.rotation() = rotationPose;
  target.translation() = translationPose + delta_lineW;
  targetVelocity.linear() = delta_lineW / ctl.timeStep;

  ctl.rightHandTask->targetPose(target);

  ctl.comQP().updateRHPose(target);
  ctl.comQP().updateRHVelocity(targetVelocity);
  
  ctl.frictionEstimator_rh.update(ctl.robot());
  //double EstimatedFriction = ctl.frictionEstimator_rh.mu_calc();

  //mc_rtc::log::info("EstimatedFriction: {}", EstimatedFriction);
  ctl.setTargetFromCoMQP();
  //ctl.setFeetTargetFromCoMQP();
  // handForceFilter_.add(ctl.robot().forceSensor("RightHandForceSensor").worldWrench(ctl.robot()));

  if (tune_){
    return false;
  }
  else{
    output("OK");
    return true;
  }
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
  removeTunningGUI(ctl);
}


// rightHandTask->admittance(sva::ForceVecd({0, 0, 0}, {0, 0, 1e-3}));

void WipingController_WipeItBaby_rh::addTunningGUI(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);
  mc_rtc::log::info("Initial admittance is: {} and {}", admittanceZ_, ctl.rightHandTask->admittance().force()(3));
  ctl.gui()->addElement({categoryName_, "Gains"},
			mc_rtc::gui::NumberSlider("Admittance f_z",
						  [this](){ return this->admittanceZ_;},
						  [this, &ctl]( double v ){
						    this->admittanceZ_ = v;
						    ctl.rightHandTask->admittance(sva::ForceVecd({0, 0, 0}, {0, 0, v}));
						  },
						  0.0001, 0.003),
			mc_rtc::gui::Button("Done", [this](){this->tune_ = false;})
			);
  using Color = mc_rtc::gui::Color;
  ctl.gui()->addPlot(
		     "Normal Force Tracking",
		     mc_rtc::gui::plot::X("t", [this](){return t_;}),
		     mc_rtc::gui::plot::Y("Normal Force (Measure)", [&ctl](){return ctl.rightHandTask->measuredWrench().force()(2);}, Color::Red),
		     mc_rtc::gui::plot::Y("Normal Force (Target", [&ctl](){return ctl.rightHandTask->targetWrench().force()(2);}, Color::Blue)
		     
		     );
  ctl.gui()->addElement({categoryName_, "Trajectory"},
			mc_rtc::gui::Label("Target Normal Force", [&ctl](){return ctl.rightHandTask->targetWrench().force()(2);})
			);
}

void WipingController_WipeItBaby_rh::removeTunningGUI(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeCategory({categoryName_});
  ctl.gui()->removePlot("Normal Force Tracking");
}

void WipingController_WipeItBaby_rh::resetTrajectory()
{
  wipingTime_ = 0.0;
  // store the initial values for local_x and local_y
  assert(false);
}

void WipingController_WipeItBaby_rh::startResumeTrajectory()
{
  Wiping_ = true;
}

void WipingController_WipeItBaby_rh::pauseTrajectory()
{
  Wiping_ = false;
}

void WipingController_WipeItBaby_rh::updateTrajectory(double timeStep)
{
  local_x_prev = local_x;
  local_y_prev = local_y;
  
  if (!Wiping_) return;
  
  wipingTime_ += timeStep;
  double timeRatio = wipingTime_/wipingDuration_;
  
  if(linearWiping_){ // goes up and comes back
    double offset = sin(timeRatio * M_PI) * amplitude_;
    local_x = local_x_initial + cos(orientation_)*offset;
    local_y = local_y_initial + sin(orientation_)*offset;
  }
  else if(circleWiping_CCW_ || circleWiping_CW_)
  {
    double Ox, Oy, offsetX, offsetY;
    Ox = local_x_initial + cos(orientation_)*amplitude_;
    Oy = local_y_initial + sin(orientation_)*amplitude_;
    if(circleWiping_CCW_){
      offsetX = amplitude_ * cos(2*M_PI*timeRatio-orientation_);
      offsetY = amplitude_ * sin(2*M_PI*timeRatio-orientation_);
    }
    else if(circleWiping_CW_){
      offsetX = amplitude_ * cos(-2*M_PI*timeRatio-orientation_);
      offsetY = amplitude_ * sin(-2*M_PI*timeRatio-orientation_);
    }
    local_x = Ox + offsetX;
    local_y = Oy + offsetY;
  }
}

EXPORT_SINGLE_STATE("WipingController_WipeItBaby_rh", WipingController_WipeItBaby_rh)


