#include "WipingController_PrepareWiping.h"

#include "../WipingController.h"

void WipingController_PrepareWiping::configure(const mc_rtc::Configuration & config)
{
  if(config.has("desiredNormalForce"))
  {
    desiredNormalForce_ = config("desiredNormalForce");
  }
  autoTransition_ = config("autoTransition", false);
	if(config.has("admittance"))
	{
	  admittance_ = config("admittance");
	}
}

void WipingController_PrepareWiping::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);

  ctl.comTask->weight(2000);
  ctl.comTask->stiffness(100);
  ctl.comTask->reset();

  ctl.lookAtTask->stiffness(5);
  ctl.lookAtTask->weight(10);
  //ctl.solver().addTask(ctl.lookAtTask);

  ctl.admittanceTask->reset();
  // Eigen::Vector6d dimW;
  // dimW << 1., 1., 1., 1., 1., 1.;
  // ctl.admittanceTask->dimWeight(dimW);
  ctl.admittanceTask->admittance(admittance_);
	ctl.admittanceTask->targetCoP({0.,-0.02});
  // double s = 1.;
  // double d = 300.;
  // ctl.admittanceTask->damping(sva::MotionVecd({d,d,d},{0.,0.,d}));
  // ctl.admittanceTask->stiffness(sva::MotionVecd({s,s,s},{10.0,10.0,s}));
  ctl.comQP().desiredNormalForce(desiredNormalForce_);
  ctl.setTargetFromCoMQP();
  ctl.setFeetTargetFromCoMQP();

  ctl.solver().addTask(ctl.comTask);
	ctl.addHandForceControl();
	ctl.addLeftFootForceControl();
  ctl.comQP().addToLogger(ctl.logger());
}

bool WipingController_PrepareWiping::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);

  ctl.setTargetFromCoMQP();
  ctl.setFeetTargetFromCoMQP();

  if(
			(ctl.admittanceTask->measuredWrench().couple() - ctl.admittanceTask->targetWrench().couple()).norm() < 1
		  && std::fabs(ctl.admittanceTask->measuredWrench().force().z() - ctl.admittanceTask->targetWrench().force().z()) < 1
     && ctl.comTask->eval().x() < 0.005)
  {
    LOG_SUCCESS("Wiping ready");
    output("OK");
    return autoTransition_;
  }

  return false;
}

void WipingController_PrepareWiping::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<WipingController &>(ctl_);
	ctl.removeHandForceControl();
  ctl.solver().removeTask(ctl.comTask);
  ctl.comQP().removeFromLogger(ctl.logger());
}

EXPORT_SINGLE_STATE("WipingController_PrepareWiping", WipingController_PrepareWiping)
