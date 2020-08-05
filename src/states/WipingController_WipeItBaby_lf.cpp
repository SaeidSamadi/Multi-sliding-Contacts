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
}

void WipingController_WipeItBaby_lf::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);


 // if(feetForceControl_)
 // {
 //   ctl.addFootForceControl();
 // }
 // else
 // {
 //   ctl.removeFootForceControl();
 // }
  ctl.comTask->reset();
  ctl.comTask->weight(2000);
  ctl.comTask->stiffness(60);

 // ctl.lookAtTask->stiffness(5);
 // ctl.lookAtTask->weight(10);
 // ctl.solver().addTask(ctl.lookAtTask);

  ctl.leftFootTask->reset();
  ctl.leftFootTask->setGains(1, 300);
  //ctl.leftFootTask->stiffness(1.);
  //ctl.leftFootTask->damping(300.);
  Eigen::Vector6d dimW;
  dimW << 1., 1., 1., 0., 0., 1.;
  ctl.leftFootTask->dimWeight(dimW);
  ctl.leftFootTask->admittance(admittance_);
  ctl.leftFootTask->targetCoP(Eigen::Vector2d::Zero());
  ctl.setTargetFromCoMQP();
  ctl.addLeftFootForceControl();

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

bool WipingController_WipeItBaby_lf::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);

  ctl.setTargetFromCoMQP();
  //ctl.setFeetTargetFromCoMQP();
  // handForceFilter_.add(ctl.robot().forceSensor("LeftFootForceSensor").worldWrench(ctl.robot()));

  output("OK");
  return true;
}

void WipingController_WipeItBaby_lf::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);
  ctl.removeLeftFootForceControl();
  //ctl.removeLeftFootForceControl();
  //ctl.removeFootForceControl();
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
