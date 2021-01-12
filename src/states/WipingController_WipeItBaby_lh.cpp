#include "WipingController_WipeItBaby_lh.h"

#include "../WipingController.h"

void WipingController_WipeItBaby_lh::configure(const mc_rtc::Configuration & config)
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

void WipingController_WipeItBaby_lh::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);

  stiffness_ = ctl.leftHandTask->mvStiffness().vector();
  damping_ = ctl.leftHandTask->mvDamping().vector();
  
  // check if some tuned gains are stored by the controller:
  if (ctl.datastore().get<bool> ("hasTunedGains_lh"))
    { 
      auto & tunedGains = ctl.datastore().get<mc_rtc::Configuration> ("TunedGains_lh");
      loadTunedGainsFromConf(tunedGains);
    }
  else
    {
      mc_rtc::log::error("[WipingController_WipeItBaby_lh] There is no tuned gains for the Left Hand");
    }
  mc_rtc::log::info("[WipingController_WipeItBaby_lh] The admittance gains are: {}", admittance_.transpose());
  mc_rtc::log::info("[WipingController_WipeItBaby_lh] The stiffness gains are: {}", stiffness_.transpose());
  mc_rtc::log::info("[WipingController_WipeItBaby_lh] The damping gains are: {}", damping_.transpose());
  
  t_ = 0.0;
  if (tune_){
    mc_rtc::log::warning("[WipingController_WipeItBaby_lh] Tunning Mode Activated!");
    Wiping_ = false;
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

  ctl.leftHandTask->reset();
  Eigen::Vector6d dimW;
  dimW << 1., 1., 1., 1., 1., 1.;
  sva::MotionVecd stiffnessGain(stiffness_), dampingGain(damping_);
  // stiffnessGain.angular() << 10, 10, 10;
  // stiffnessGain.linear() << 10, 10, 5;
  // dampingGain.angular() << 6, 6, 6;
  // dampingGain.linear() << 6, 6, 300;
  ctl.leftHandTask->setGains(stiffnessGain, dampingGain);
  ctl.leftHandTask->dimWeight(dimW);
  ctl.leftHandTask->admittance(admittance_);
  ctl.leftHandTask->targetCoP(Eigen::Vector2d::Zero());
  ctl.leftHandTask->targetPose();
  ctl.setTargetFromCoMQP();
  ctl.addLeftHandForceControl();

  //ctl.setFeetTargetFromCoMQP();
  //ctl.addLeftFootForceControl();
  ctl.solver().addTask(ctl.comTask);

  ctl.comQP().addToLogger(ctl.logger());

  ctl.logger().addLogEntry("friction_mu_Estim_lh",
                           [&ctl]()
                           {
                           return ctl.frictionEstimator_lh.mu_calc();
                           });
  ctl.logger().addLogEntry("friction_mu_filtered_lh",
                           [&ctl]()
                           {
                           return ctl.frictionEstimator_lh.mu_filtered();
                           });

    ctl.logger().addLogEntry("SurfaceWrench_lh_x",
                           [&ctl]()
                           {
                           return ctl.frictionEstimator_rh.forceX();
                           });
  ctl.logger().addLogEntry("SurfaceWrench_lh_y",
                           [&ctl]()
                           {
                           return ctl.frictionEstimator_rh.forceY();
                           });
  ctl.logger().addLogEntry("SurfaceWrench_lh_z",
                           [&ctl]()
                           {
                           return ctl.frictionEstimator_rh.forceZ();
                           });
}

bool WipingController_WipeItBaby_lh::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);

  t_ += ctl.timeStep;
  updateTrajectory(ctl.timeStep);

  delta_line << local_x - local_x_prev,
                local_y - local_y_prev,
                0.0;
  
  delta_lineW = ctl.wallPosInvW * delta_line;
  sva::PTransformd poseOutput = ctl.leftHandTask->targetPose();
  auto rotationPose = poseOutput.rotation();
  auto translationPose = poseOutput.translation();
  sva::PTransformd target;
  sva::MotionVecd targetVelocity = sva::MotionVecd::Zero();
  target.rotation() = rotationPose;
  target.translation() = translationPose + delta_lineW;
  targetVelocity.linear() = delta_lineW / ctl.timeStep;

  ctl.leftHandTask->targetPose(target);

  ctl.comQP().updateLHPose(target);
  ctl.comQP().updateLHVelocity(targetVelocity);
  
  ctl.frictionEstimator_lh.update(ctl.robot());
  
  ctl.setTargetFromCoMQP();

  if (wipingTime_ >= wipingDuration_ and !tune_)
    {
      Wiping_ = false;
    }
  
  if (tune_){
    return false;
  }
  else{
    output("OK");
    return true;
  }
}

void WipingController_WipeItBaby_lh::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);
  ctl.removeLeftHandForceControl();
  //ctl.removeLeftFootForceControl();
  ctl.removeFootForceControl();
  ctl.solver().removeTask(ctl.comTask);
  ctl.solver().removeTask(ctl.lookAtTask);

  ctl.lookAtTask->stiffness(1);
  ctl.lookAtTask->weight(1);

  ctl.comQP().removeFromLogger(ctl.logger());

  ctl.logger().removeLogEntry("friction_mu_x");
  ctl.logger().removeLogEntry("friction_mu_y");
  ctl.logger().removeLogEntry("friction_mu");
  ctl.logger().removeLogEntry("friction_mu_Estim_lh");
  ctl.logger().removeLogEntry("friction_mu_filtered_lh");
  ctl.logger().removeLogEntry("SurfaceWrench_lh_x");
  ctl.logger().removeLogEntry("SurfaceWrench_lh_y");
  ctl.logger().removeLogEntry("SurfaceWrench_lh_z");  
  
  removeTunningGUI(ctl);
  saveTunedGains();

  auto & hasTunedGains = ctl.datastore().get<bool> ("hasTunedGains_lh");
  hasTunedGains = true;
  auto & tunedGains = ctl.datastore().get<mc_rtc::Configuration> ("TunedGains_lh");
  tunedGains = mc_rtc::Configuration();
  addTunedGainsToConf(tunedGains);
}

void WipingController_WipeItBaby_lh::addTunningGUI(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);
  ctl.gui()->addElement({categoryName_, "Gains"},
			//tunning of the admittance gains
			mc_rtc::gui::NumberSlider("Admittance Gain f_z",
						  [this](){ return admittance_(5);},
						  [this, &ctl]( double v ){
						    admittance_(5) = v;
						    ctl.leftHandTask->admittance(admittance_);
						  },
						  0.0001, 0.004),
			mc_rtc::gui::ArrayInput("Left Hand Admittance Gains", {},
						[this](){return admittance_;},
						[this, &ctl] ( const Eigen::Vector6d & v){
						  admittance_ = v;
						  ctl.leftHandTask->admittance(admittance_);
						}),
			// tunning of the stiffness gains
			mc_rtc::gui::NumberSlider("Admittance Stiffness f_z",
						  [this](){return stiffness_(5);},
						  [this, &ctl]( double v ){
						    stiffness_(5) = v;
						    double dampingZ = ctl.leftHandTask->mvDamping().vector()(5);
						    sva::MotionVecd stiff(stiffness_);
						    ctl.leftHandTask->stiffness(stiff);
						    damping_ = ctl.leftHandTask->mvDamping().vector();
						    damping_(5) = dampingZ;
						    sva::MotionVecd damp(damping_);
						    ctl.leftHandTask->damping(damp);
						  },
						  0.0, 12.0),
			mc_rtc::gui::ArrayInput("Left Hand Admittance Stiffness Gains", {},
						[this](){ return stiffness_;},
						[this, &ctl]( const Eigen::Vector6d v ){
						  stiffness_ = v;
						  double dampingZ = ctl.leftHandTask->mvDamping().vector()(5);
						  sva::MotionVecd stiff(stiffness_);
						  ctl.leftHandTask->stiffness(stiff);
						  damping_ = ctl.leftHandTask->mvDamping().vector();
						  damping_(5) = dampingZ;
						  sva::MotionVecd damp(damping_);
						  ctl.leftHandTask->damping(damp);
						}),
			// tunning of the damping gains
			mc_rtc::gui::NumberSlider("Admittance Damping f_z",
						  [this](){ return damping_(5); },
						  [this, & ctl]( double v ){
						    damping_(5) = v;
						    sva::MotionVecd damp(damping_);
						    ctl.leftHandTask->damping(damp);
						  },
						  0.0, 400.0),
			mc_rtc::gui::ArrayInput("Left Hand Admittance Damping Gains", {},
						[this, &ctl](){
						  damping_ = ctl.leftHandTask->mvDamping().vector();
						  return damping_; },
						[this, &ctl]( const Eigen::Vector6d v){
						    damping_ = v;
						    sva::MotionVecd damp(damping_);
						    ctl.leftHandTask->damping(damp);
						  }),
			// other stuff
			mc_rtc::gui::Button("Start/Stop", [this](){Wiping_ = !Wiping_;}),
			mc_rtc::gui::Button("Done", [this](){this->tune_ = false;})
			);
  using Color = mc_rtc::gui::Color;
  ctl.gui()->addPlot(
		     "Left Hand Normal Force Tracking",
		     mc_rtc::gui::plot::X("t", [this](){return t_;}),
		     mc_rtc::gui::plot::Y("Normal Force (Measure)", [&ctl](){return ctl.leftHandTask->measuredWrench().force()(2);}, Color::Red),
		     mc_rtc::gui::plot::Y("Normal Force (Target", [&ctl](){return ctl.leftHandTask->targetWrench().force()(2);}, Color::Blue)
		     
		     );
  ctl.gui()->addPlot(
		     "Left Hand Trajectory Tracking",
		     mc_rtc::gui::plot::X("t", [this](){return t_;}),
		     mc_rtc::gui::plot::Y("Trajectory X (Target)", [&ctl](){return ctl.leftHandTask->targetPose().translation().x();}, Color::Blue),
		     mc_rtc::gui::plot::Y("Trajectory X (Measure)", [&ctl](){return ctl.leftHandTask->surfacePose().translation().x();}, Color::Red),
		     mc_rtc::gui::plot::Y("Trajectory Y (Target)", [&ctl](){return ctl.leftHandTask->targetPose().translation().y();}, Color::Green),
		     mc_rtc::gui::plot::Y("Trajectory Y (Measure)", [&ctl](){return ctl.leftHandTask->surfacePose().translation().y();}, Color::Magenta)
		     );
  ctl.gui()->addElement({categoryName_, "Trajectory"},
			mc_rtc::gui::Label("Target Normal Force", [&ctl](){return ctl.leftHandTask->targetWrench().force()(2);})
			);
}

void WipingController_WipeItBaby_lh::removeTunningGUI(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeCategory({categoryName_});
  ctl.gui()->removePlot("Left Hand Normal Force Tracking");
  ctl.gui()->removePlot("Left Hand Trajectory Tracking");
}

void WipingController_WipeItBaby_lh::resetTrajectory()
{
  wipingTime_ = 0.0;
  // store the initial values for local_x and local_y
  assert(false);
}

void WipingController_WipeItBaby_lh::startResumeTrajectory()
{
  Wiping_ = true;
}

void WipingController_WipeItBaby_lh::pauseTrajectory()
{
  Wiping_ = false;
}

void WipingController_WipeItBaby_lh::updateTrajectory(double timeStep)
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
    Ox = local_x_initial - cos(orientation_)*amplitude_;
    Oy = local_y_initial - sin(orientation_)*amplitude_;
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

void WipingController_WipeItBaby_lh::addTunedGainsToConf( mc_rtc::Configuration & config)
{
  config.add("admittance", admittance_);
  config.add("stiffness", stiffness_);
  config.add("damping", damping_);
}

void WipingController_WipeItBaby_lh::loadTunedGainsFromConf( mc_rtc::Configuration & config)
{
  if(config.has("admittance")) admittance_ = config("admittance");
  if (config.has("stiffness")) stiffness_ = config("stiffness");
  if (config.has("damping")) damping_ = config("damping");
}

void WipingController_WipeItBaby_lh::saveTunedGains()
{
  mc_rtc::Configuration confSave;
  auto section = confSave.add("TunedGains_lh");
  addTunedGainsToConf(section);
  confSave.save("/tmp/tunedGains_lh.yaml");
}

EXPORT_SINGLE_STATE("WipingController_WipeItBaby_lh", WipingController_WipeItBaby_lh)
