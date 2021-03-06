#include "WipingController.h"

WipingController::WipingController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config), comQP_(robot(), config("CoMQPConfig")),
    frictionEstimator_rh(robot(), "RightHandPad", Eigen::Vector3d{0.,0.,-1.}, config("CoMQPConfig")("right_hand")("friction")),
    frictionEstimator_lh(robot(), "BlockLeftHand", Eigen::Vector3d{0.,0.,-1.}, config("CoMQPConfig")("left_hand")("friction")),
  frictionEstimator_lf(robot(), "LeftFoot", Eigen::Vector3d{0.,0.,-1.}, config("CoMQPConfig")("left_foot")("friction")), t_(0.0)
{ 
  useFeetForceControl_ = config("UseFeetForceControl", false);

  auto & hasTunedGains_rh = datastore().make<bool> ("hasTunedGains_rh", false);
  auto & tunedGains_rh = datastore().make_initializer<mc_rtc::Configuration> ("TunedGains_rh");

  if (config.has("TunedGains_rh"))
    {
      mc_rtc::log::warning("[Wiping Controller] Found Tunned gains in the configuration for the right hand");
      hasTunedGains_rh = true;
      tunedGains_rh = config("TunedGains_rh");
    }
  else
    {
      mc_rtc::log::warning("[Wiping Controller] Right Hand Tunned gains not Found!");
    }

  auto & hasTunedGains_lh = datastore().make<bool> ("hasTunedGains_lh", false);
  auto & tunedGains_lh = datastore().make_initializer<mc_rtc::Configuration> ("TunedGains_lh");

  if (config.has("TunedGains_lh"))
    {
      mc_rtc::log::warning("[Wiping Controller] Found Tunned gains in the configuration for the left hand");
      hasTunedGains_lh = true;
      tunedGains_lh = config("TunedGains_lh");
    }
  else
    {
      mc_rtc::log::warning("[Wiping Controller] Left Hand Tunned gains not Found!");
    }

  comTask.reset(new mc_tasks::CoMTask(robots(), robots().robotIndex(), 5, 1000));

  rightHandTask.reset(new mc_tasks::force::CoPTask("RightHandPad", robots(), robots().robotIndex(), 5., 500));//Stiff, Weight
  rightHandTask->setGains(1, 300);
  rightHandTask->admittance(sva::ForceVecd({0, 0, 0}, {0, 0, 1e-3}));
  leftHandTask.reset(new mc_tasks::force::CoPTask("BlockLeftHand", robots(), robots().robotIndex(), 5., 500));
  leftHandTask->setGains(1, 300);
  leftHandTask->admittance(sva::ForceVecd({0, 0, 0}, {0, 0, 1e-3}));


  leftFootTask.reset(new mc_tasks::force::CoPTask("LeftFootCenter", robots(), robots().robotIndex(), 100, 1000));
  rightFootTask.reset(new mc_tasks::force::CoPTask("RightFootCenter", robots(), robots().robotIndex(), 100, 10000));

  lookAtTask.reset(new mc_tasks::LookAtSurfaceTask(robots(), robots().robotIndex(), "xtion_link", {1., 0., 0.},
                                                   robots().robotIndex(), "RightHandPad", 1.0, 10.));
  //solver().addTask(lookAtTask);

  tiltedboardPosInvW = robots().robot("tilted_board").posW().rotation().inverse();
  wallPosInvW = robots().robot("wall").posW().rotation().inverse();
  slopePosInvW = robots().robot("slope").posW().rotation().inverse();

  auto handForceConfig = mc_rtc::gui::ForceConfig(mc_rtc::gui::Color(0., 1., 0.));
  handForceConfig.force_scale *= 3;
  gui()->addElement(
      {"WipingController", "Markers", "Forces"},
      mc_rtc::gui::Force("RightHandForce", handForceConfig,
                         [this]() { return robot().forceSensor("RightHandForceSensor").worldWrench(robot()); },
                         [this]() { return robot().surface("RightHandPad").X_0_s(robot()); }),
      mc_rtc::gui::Force("LeftHandForce", handForceConfig,
                         [this]() { return robot().forceSensor("LeftHandForceSensor").worldWrench(robot()); },
                         [this]() { return robot().surface("BlockLeftHand").X_0_s(robot()); }),
      mc_rtc::gui::Force("RightFootForce", mc_rtc::gui::ForceConfig(mc_rtc::gui::Color(1., 0.2, 0.)),
                         [this]() { return robot().forceSensor("RightFootForceSensor").wrenchWithoutGravity(robot()); },
                         [this]() { return robot().surface("RightFoot").X_0_s(robot()); }),
      mc_rtc::gui::Force("LeftFootForce", mc_rtc::gui::ForceConfig(mc_rtc::gui::Color(1., 0.2, 0.)),
                         [this]() { return robot().forceSensor("LeftFootForceSensor").wrenchWithoutGravity(robot()); },
                         [this]() { return robot().surface("LeftFoot").X_0_s(robot()); }));

  gui()->addElement(
      {"CoM"},
      mc_rtc::gui::Point3D("CoM", mc_rtc::gui::PointConfig(mc_rtc::gui::Color{1., 1., 0.}, 0.04),
                           [this]() {
                             const Eigen::Vector3d com = robot().com();
                             return Eigen::Vector3d{com.x(), com.y(), 0.};
                           }),
      mc_rtc::gui::Point3D("CoMTarget", mc_rtc::gui::PointConfig(mc_rtc::gui::Color{1., 0., 0.}, 0.04), [this]() {
        const Eigen::Vector3d com = comTask->com();
        return Eigen::Vector3d{com.x(), com.y(), 0.};
      }));

  comQP_.addToGUI(*gui());

  logger().addLogEntry("perf_CoMQP",
                       [this]()
                       {
                        return comqp_dt_.count();
                       });
  logger().addLogEntry("compp_real",
		       [this](){
			 return realRobot("hrp4").comAcceleration();});
  logger().addLogEntry("compp_sim",
		       [this](){
			 return robot("hrp4").comAcceleration();});
  logger().addLogEntry("comp_real",
		       [this](){
			 return realRobot("hrp4").comVelocity();});
  logger().addLogEntry("comp_sim",
		       [this](){
			 return robot("hrp4").comVelocity();});
  
  
  datastore().make_call("KinematicAnchorFrame::" + robot().name(), [this](const mc_rbdyn::Robot & robot) {
    return sva::interpolate(robot.surfacePose("LeftFoot"), robot.surfacePose("RightFoot"), 0.5);
  });

  //logger().addLogEntry("RightHandPose", [this]() {sva::PTransformd x;
  //                                                x=robot().surface("RightHandPad").X_0_s(robot());
  //                                                return x; });
  //logger().addLogEntry("RightHandVelocity", [this]() { sva::MotionVecd rhVel;
  //                                                   rhVel = robot().bodyVelW("r_wrist"); return rhVel; });
  //addFootForceControl();

  using Color = mc_rtc::gui::Color;
  gui()->addPlot(
		 "CoM Trajectory Tracking",
		 mc_rtc::gui::plot::X("t", [this](){return t_;}),
		 mc_rtc::gui::plot::Y("CoM Target X", [this](){return comTask->com().x();}, Color::Blue),
		 mc_rtc::gui::plot::Y("CoM Real X", [this](){return realRobot("hrp4").com().x();}, Color::Red),
		 mc_rtc::gui::plot::Y("CoM Target Y", [this](){return comTask->com().y();}, Color::Green),
		 mc_rtc::gui::plot::Y("CoM Real Y", [this](){return realRobot("hrp4").com().y();}, Color::Magenta)
		 );

  gui()->addPlot("Left Foot Force Tracking",
		 mc_rtc::gui::plot::X("t", [this](){return t_;}),
		 mc_rtc::gui::plot::Y("Normal Force Target", [this](){return leftFootTask->targetForce().z();}, Color::Blue),
		 mc_rtc::gui::plot::Y("Normal Force Measure", [this](){return leftFootTask->measuredWrench().force().z();}, Color::Red)
		 );


  
  mc_rtc::log::success("WipingController init done!");
}

bool WipingController::computeCoMQP()
{
  //Eigen::Vector2d muYZ = this->comQP().muYZ_rh();
  //double mu_y = muYZ.x();
  //double mu_z = muYZ.y();
  //comQP().updateNumVar(robot());
  comQPComputed = this->comQP().solve(this->robot(), frictionEstimator_rh.mu_filtered(), frictionEstimator_lh.mu_filtered(), frictionEstimator_lf.mu_filtered());
  if(!comQPComputed)
  {
    LOG_ERROR("CoMQP Failed to run with error code " << this->comQP().errorCode() << ":");
    this->comQP().errorMessage();
    return false;
  }
  else
  {
    if(this->comQP().debug())
    {
      const Eigen::VectorXd & resultVector = this->comQP().resultVector();
      LOG_INFO("Result: " << resultVector.transpose());
    }
  }
  return true;
}

void WipingController::setTargetFromCoMQP()
{
  if(comQPComputed)
  {
    const auto & result = this->comQP().result();
    this->comTask->com(Eigen::Vector3d{result.comPos(0), result.comPos(1), comHeight_});
    this->rightHandTask->targetForceW(result.rightHandForce.force());
    this->leftHandTask->targetForceW(result.leftHandForce.force());
    this->leftFootTask->targetForceW(result.leftFootForce.force());
  }
  else
  {
    LOG_WARNING("CoMQP Failed to compute, ignoring target");
  }
}

void WipingController::setFeetTargetFromCoMQP()
{
  if(comQPComputed)
  {
    const auto & result = this->comQP().result();
    this->leftFootTask->targetForceW(result.leftFootForce.force());
    this->rightFootTask->targetForceW(result.rightFootForce.force());

    if(useFeetForceControl_)
    {
      updateFootForceDifferenceControl();
    }
  }
  else
  {
    LOG_WARNING("CoMQP Failed to compute, ignoring target II");
  }
}

void WipingController::addRightHandForceControl()
{
  solver().addTask(rightHandTask);
  gui()->addElement(
      {"WipingController", "Markers", "Forces"},
      mc_rtc::gui::Point3D("RightHandCoP",
                           [this]() { return robot().copW("RightHandPad"); }),
      mc_rtc::gui::Point3D("RightHandCoPTarget",
                           mc_rtc::gui::PointConfig(mc_rtc::gui::Color{0.,1.,0.}),
                           [this]() { return rightHandTask->targetCoPW(); })
      );
}

void WipingController::removeRightHandForceControl()
{
  solver().removeTask(rightHandTask);
  gui()->removeElement({"WipingController", "Markers", "Forces"}, "RightHandCoP");
  gui()->removeElement({"WipingController", "Markers", "Forces"}, "RightHandCoPTarget");
}

void WipingController::addLeftHandForceControl()
{
  solver().addTask(leftHandTask);
  gui()->addElement(
      {"WipingController", "Markers", "Forces"},
      mc_rtc::gui::Point3D("LeftHandCoP",
                           [this]() { return robot().copW("BlockLeftHand"); }),
      mc_rtc::gui::Point3D("LeftHandCoPTarget",
                           mc_rtc::gui::PointConfig(mc_rtc::gui::Color{0.,1.,0.}),
                           [this]() { return leftHandTask->targetCoPW(); })
      );
}

void WipingController::removeLeftHandForceControl()
{
  solver().removeTask(leftHandTask);
  gui()->removeElement({"WipingController", "Markers", "Forces"}, "LeftHandCoP");
  gui()->removeElement({"WipingController", "Markers", "Forces"}, "LeftHandCoPTarget");
}

void WipingController::addLeftFootForceControl()
{
  solver().addTask(leftFootTask);
  gui()->addElement(
      {"WipingController", "Markers", "Forces"},
      mc_rtc::gui::Point3D("LeftFootCoP",
                           [this]() { return robot().copW("LeftFootCenter"); }),
      mc_rtc::gui::Point3D("LeftFootCoPTarget",
                           mc_rtc::gui::PointConfig(mc_rtc::gui::Color{0.,1.,0.}),
                           [this]() { return leftFootTask->targetCoPW(); })
      );
}

void WipingController::removeLeftFootForceControl()
{
  solver().removeTask(leftFootTask);
  gui()->removeElement({"WipingController", "Markers", "Forces"}, "LeftFootCoP");
  gui()->removeElement({"WipingController", "Markers", "Forces"}, "LeftFootCoPTarget");
}
void WipingController::addFootForceControl()
{
  if(useFeetForceControl_)
  {
    solver().addTask(rightFootTask);
    solver().addTask(leftFootTask);

    // Eigen::Vector6d dof;
    // dof << 1,1,1,1,1,1;
    // addContact({"hrp4", "ground", "RightFoot", "AllGround", 0.5, dof});
    // addContact({"hrp4", "ground", "LeftFoot", "AllGround", 0.5, dof});
    mc_rtc::log::success("FeetForceControlAdded");
  }
  else
  {
    LOG_WARNING("Requesting to use foot force control but this feature is disable in the configuration. Please use \"UseFeetForceControl: true\"");
  }
}

void WipingController::removeFootForceControl()
{
  solver().removeTask(rightFootTask);
  solver().removeTask(leftFootTask);
  // Eigen::Vector6d dof;
  // dof << 1,1,1,1,1,1;
  // addContact({"hrp4", "ground", "RightFoot", "AllGround", 0.5, dof});
  // addContact({"hrp4", "ground", "LeftFoot", "AllGround", 0.5, dof});
  mc_rtc::log::success("FeetForceControl removed");
}

void WipingController::updateFootForceDifferenceControl()
{
  leftFootTask->targetPose(lfTarget_);
  rightFootTask->targetPose(rfTarget_);

  double LFz = leftFootTask->measuredWrench().force().z();
  double RFz = rightFootTask->measuredWrench().force().z();
  double LFz_d = leftFootTask->targetWrench().force().z();
  double RFz_d = rightFootTask->targetWrench().force().z();
  double dz_ctrl = dfzAdmittance_ * ((LFz_d - RFz_d) - (LFz - RFz));

  double LTz = leftFootTask->surfacePose().translation().z();
  double RTz = rightFootTask->surfacePose().translation().z();
  double vfcZCtrl_ = RTz - LTz;
  dz_ctrl -= vdcDamping_ * vfcZCtrl_;

  double LTz_d = leftFootTask->targetPose().translation().z();
  double RTz_d = rightFootTask->targetPose().translation().z();
  double dz_pos = vdcFrequency_ * ((LTz_d + RTz_d) - (LTz + RTz));
  double vdcZPos_ = RTz + LTz;

  sva::MotionVecd velF = {{0., 0., 0.}, {0., 0., dz_ctrl}};
  sva::MotionVecd velT = {{0., 0., 0.}, {0., 0., dz_pos}};
  leftFootTask->refVelB(0.5 * (velT - velF));
  rightFootTask->refVelB(0.5 * (velT + velF));
}

bool WipingController::run()
{
  /** Always pick a steady clock */
  using clock = typename std::conditional<std::chrono::high_resolution_clock::is_steady,
                                          std::chrono::high_resolution_clock, std::chrono::steady_clock>::type;
  auto start_run_t = clock::now();

  t_ += timeStep;

  realLeftFootPose = robot().surface("LeftFoot").X_0_s(robot());
  //leftFootPose = robot().surface("RightHandPad").X_0_s(robot()).translation();
  //leftFootRot = robot().surface("LeftFoot").X_0_s(robot()).rotation();
  //mc_rtc::log::info("real.leftFootPose: \n {}", leftFootPose);
  computeCoMQP();
  comqp_dt_ = clock::now() - start_run_t;
  setFeetTargetFromCoMQP();
  return mc_control::fsm::Controller::run();
}

WipingController::~WipingController(){
  logger().removeLogEntry("RightHandPose");
};

void WipingController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
  comTask->reset();
  comHeight_ = robot().com().z();

  leftFootTask->reset();
  leftFootTask->setGains(1, 300);
  leftFootTask->admittance(sva::ForceVecd({0, 0, 0}, {0, 0, 1e-4}));
  rightFootTask->reset();
  rightFootTask->setGains(1, 300);
  rightFootTask->admittance(sva::ForceVecd({0, 0, 0}, {0, 0, 1e-4}));
  lfTarget_ = leftFootTask->targetPose();
  rfTarget_ = rightFootTask->targetPose();

  std::vector<mc_rbdyn::Contact> feetContacts;
  for(const auto & contact : solver().contacts())
  {
    if(contact.r1Surface()->name() == "RightFoot" || contact.r1Surface()->name() == "LeftFoot")
    {
      feetContacts.push_back(contact);
    }
  }
  if(!feetContacts.empty())
  {
  }
  else
  {
    LOG_ERROR("No foot contact in controller, cannot compute support polygon");
  }
}
