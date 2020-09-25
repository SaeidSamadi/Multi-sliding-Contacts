#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_tasks/CoPTask.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/LookAtSurfaceTask.h>

#include "CoMQP.h"
#include "FrictionEstimator.h"

struct MC_CONTROL_DLLAPI WipingController : public mc_control::fsm::Controller
{
  WipingController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);
  ~WipingController();
  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;
  
  void createLinearTrajectory();
  bool computeCoMQP();
  void setTargetFromCoMQP();
	void setFeetTargetFromCoMQP();

  CoMQP & comQP()
  {
    return comQP_;
  }

  void addFootForceControl();
  void removeFootForceControl();

	void addRightHandForceControl();
	void removeRightHandForceControl();
	void addLeftHandForceControl();
	void removeLeftHandForceControl();
	void addLeftFootForceControl();
	void removeLeftFootForceControl();
private:
  void updateFootForceDifferenceControl();

public:
  std::shared_ptr<mc_tasks::CoMTask> comTask;
  std::shared_ptr<mc_tasks::force::CoPTask> rightHandTask;
  std::shared_ptr<mc_tasks::force::CoPTask> leftHandTask;
  std::shared_ptr<mc_tasks::LookAtSurfaceTask> lookAtTask;
  FrictionEstimator frictionEstimator;
  //sva::PTransformd leftFootPose;
  Eigen::Vector3d leftFootPose;
  Eigen::Matrix3d leftFootRot;
  std::shared_ptr<mc_tasks::force::CoPTask> leftFootTask;
  std::shared_ptr<mc_tasks::force::CoPTask> rightFootTask;
  Eigen::Matrix3d tiltedboardPosInvW, wallPosInvW, slopePosInvW;
  double dfzAdmittance_ = 1e-4; /**< Admittance for vertical foot force control */
  double vdcDamping_ = 0.; /**< Vertical Drift Compensation damping */
  double vdcFrequency_ = 1.; /**< Vertical Drift Compensation frequency */
  sva::PTransformd lfTarget_, rfTarget_;

  bool comQPComputed = false;
  sva::PTransformd realLeftFootPose;
private:
  mc_rtc::Configuration config_;
  CoMQP comQP_;

  bool useFeetForceControl_ = false;
	double comHeight_ = 0.0;

  using duration_ms = std::chrono::duration<double, std::milli>;
  duration_ms comqp_dt_;
};
