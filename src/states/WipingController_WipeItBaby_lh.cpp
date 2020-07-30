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
}

void WipingController_WipeItBaby_lh::start(mc_control::fsm::Controller & ctl_)
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

  ctl.leftHandTask->reset();
  ctl.leftHandTask->setGains(1, 300);
  //ctl.leftHandTask->stiffness(1.);
  //ctl.leftHandTask->damping(300.);
  Eigen::Vector6d dimW;
  dimW << 1., 1., 1., 0., 0., 1.;
  ctl.leftHandTask->dimWeight(dimW);
  ctl.leftHandTask->admittance(admittance_);
  ctl.leftHandTask->targetCoP(Eigen::Vector2d::Zero());
  ctl.setTargetFromCoMQP();
  ctl.addRightHandForceControl();

  //ctl.setFeetTargetFromCoMQP();
  //ctl.addLeftFootForceControl();
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

bool WipingController_WipeItBaby_lh::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);

  ctl.setTargetFromCoMQP();
  //ctl.setFeetTargetFromCoMQP();
  // handForceFilter_.add(ctl.robot().forceSensor("RightHandForceSensor").worldWrench(ctl.robot()));

  output("OK");
  return true;
}

void WipingController_WipeItBaby_lh::teardown(mc_control::fsm::Controller & ctl_)
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

  ctl.logger().removeLogEntry("friction_mu_x");
  ctl.logger().removeLogEntry("friction_mu_y");
  ctl.logger().removeLogEntry("friction_mu");

}

EXPORT_SINGLE_STATE("WipingController_WipeItBaby_lh", WipingController_WipeItBaby_lh)
