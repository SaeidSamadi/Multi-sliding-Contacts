#include "WipingController.h"

WipingController::WipingController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config), comQP_(robot(), config("CoMQPConfig")),
    frictionEstimator(robot(), "RightHandPad", Eigen::Vector3d{0.,0.,-1.})
{

  useFeetForceControl_ = config("UseFeetForceControl", false);

  comTask.reset(new mc_tasks::CoMTask(robots(), robots().robotIndex(), 5, 1000));

  admittanceTask.reset(new mc_tasks::force::CoPTask("RightHandPad", robots(), robots().robotIndex(), 5., 500));
  admittanceTask->setGains(1, 300);
  admittanceTask->admittance(sva::ForceVecd({0, 0, 0}, {0, 0, 1e-3}));

  leftFootTask.reset(new mc_tasks::force::CoPTask("LeftFootCenter", robots(), robots().robotIndex(), 100000, 10000));
  rightFootTask.reset(new mc_tasks::force::CoPTask("RightFootCenter", robots(), robots().robotIndex(), 100000, 10000));

  lookAtTask.reset(new mc_tasks::LookAtSurfaceTask(robots(), robots().robotIndex(), "xtion_link", {1., 0., 0.},
                                                   robots().robotIndex(), "RightHandPad", 1.0, 10.));
  solver().addTask(lookAtTask);

  auto handForceConfig = mc_rtc::gui::ForceConfig(mc_rtc::gui::Color(0., 1., 0.));
  handForceConfig.force_scale *= 3;
  gui()->addElement(
      {"Forces"},
      mc_rtc::gui::Force("RightHandForce", handForceConfig,
                         [this]() { return robot().forceSensor("RightHandForceSensor").worldWrench(robot()); },
                         [this]() { return robot().surface("RightHandPad").X_0_s(robot()); }),
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

  logger().addLogEntry("RightHandPose", [this]() {sva::PTransformd x;
                                                  x=robot().surface("RightHandPad").X_0_s(robot());
                                                  return x; });
  addFootForceControl();
  LOG_SUCCESS("WipingController init done " << this)
}

bool WipingController::computeCoMQP()
{
  Eigen::Vector2d muYZ = this->comQP().muYZ();
  double mu_y = muYZ.x();
  double mu_z = muYZ.y();
  //this->shiftedSupportPolygon().update(this->robots(), sva::ForceVecd(Eigen::Vector3d::Zero(),
  //                                                                    {
  //                                                                    this->comQP().desiredNormalForce(),
  //                                                                    mu_y * this->comQP().desiredNormalForce(),
  //                                                                    mu_z * this->comQP().desiredNormalForce()
  //                                                                    }));

  // Solve com QP
  //comQPComputed = this->comQP().solve(this->robot(), this->shiftedSupportPolygon());
  comQPComputed = this->comQP().solve(this->robot());
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
    this->admittanceTask->targetForceW(result.rightHandForce.force());
  }
  else
  {
    LOG_WARNING("CoMQP Failed to compute, ignoring target I");
  }
}

void WipingController::setFeetTargetFromCoMQP()
{
  if(comQPComputed)
  {
    const auto & result = this->comQP().result();
    this->leftFootTask->targetForceW(result.leftFootForce.force());
    //addLeftFootForceControl();
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

void WipingController::addHandForceControl()
{
  solver().addTask(admittanceTask);
  gui()->addElement(
      {"Forces"},
      mc_rtc::gui::Point3D("RightHandCoP",
                           [this]() { return robot().copW("RightHandPad"); }),
      mc_rtc::gui::Point3D("RightHandCoPTarget",
                           mc_rtc::gui::PointConfig(mc_rtc::gui::Color{0.,1.,0.}),
                           [this]() { return admittanceTask->targetCoPW(); })
      );
}

void WipingController::removeHandForceControl()
{
  solver().removeTask(admittanceTask);
  gui()->removeElement({"Forces"}, "RightHandCoP");
  gui()->removeElement({"Forces"}, "RightHandCoPTarget");
}

void WipingController::addLeftFootForceControl()
{
  solver().addTask(leftFootTask);
 // gui()->addElement(
 //     {"Forces"},
 //     mc_rtc::gui::Point3D("LeftFootCoP",
 //                          [this]() { return robot().copW("LeftFootCenter"); }),
 //     mc_rtc::gui::Point3D("LeftFootCoPTarget",
 //                          mc_rtc::gui::PointConfig(mc_rtc::gui::Color{0.,1.,0.}),
 //                          [this]() { return leftFootTask->targetCoPW(); })
 //     );
}

void WipingController::removeLeftFootForceControl()
{
  solver().removeTask(leftFootTask);
 // gui()->removeElement({"Forces"}, "LeftFootCoP");
 // gui()->removeElement({"Forces"}, "LeftFootCoPTarget");
}
void WipingController::addFootForceControl()
{
  if(useFeetForceControl_)
  {
    solver().addTask(rightFootTask);
    solver().addTask(leftFootTask);

    Eigen::Vector6d dof;
    dof << 1,1,1,1,1,0;
    addContact({"hrp4", "ground", "RightFoot", "AllGround", 0.5, dof});
    addContact({"hrp4", "ground", "LeftFoot", "AllGround", 0.5, dof});
    LOG_SUCCESS("FeetForceControlAdded");
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
  Eigen::Vector6d dof;
  dof << 1,1,1,1,1,1;
  addContact({"hrp4", "ground", "RightFoot", "AllGround", 0.5, dof});
  addContact({"hrp4", "ground", "LeftFoot", "AllGround", 0.5, dof});
  LOG_SUCCESS("FeetForceControl removed");
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
  //supportPolygon_.update(robots());
  computeCoMQP();
  setFeetTargetFromCoMQP();
  //addLeftFootForceControl();
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
