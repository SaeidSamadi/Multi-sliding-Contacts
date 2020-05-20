#include "FrictionEstimator.h"
#include <mc_rtc/logging.h>

FrictionEstimator::FrictionEstimator(const mc_rbdyn::Robot& robot, const std::string & surfaceName, const Eigen::Vector3d & surfaceNormal)
  : surfaceName_(surfaceName),
    surfaceNormal_(surfaceNormal)
{
}

void FrictionEstimator::update(const mc_rbdyn::Robot& robot)
{
  // Direction of motion
  const auto & surface = robot.surface(surfaceName_);
  const auto & wrench = robot.surfaceWrench(surfaceName_);
  const auto & bodyName = surface.bodyName();
  const auto & bodyVel = robot.bodyVelB(bodyName);
  const auto & surfaceVel = surface.X_b_s() * bodyVel;
  Eigen::Vector3d velDir = surfaceVel.linear().normalized();

  mu_x_ = 0.;
  double N =   fabs(wrench.force().z());
  double fx =  fabs(wrench.force().x());
  double vxy = Eigen::Vector2d{ surfaceVel.linear().x(), surfaceVel.linear().y() }.norm();
  if(fx > 10e-8)
  {
    double vx =  fabs(surfaceVel.linear().x());
    mu_x_ = fx/N * vxy / vx;
  }

  mu_y_ = 0;
  double vy =  fabs(surfaceVel.linear().y());
  if(vy > 10e-8)
  {
    double fy =  fabs(wrench.force().y());
    mu_y_ = fy/N * vxy / vy;
  }

  mu_ = 0.;
  if(mu_x_ < 10e-8)
  {
    mu_ = mu_y_;
  }
  else if(mu_y_ < 10e-8)
  {
    mu_ = mu_x_;
  }
  else
  {
   mu_ = (mu_x_ + mu_y_)/2.;
  }
  // LOG_INFO("mu_x: " << mu_x_);
  // LOG_INFO("mu_y: " << mu_y_);
  // LOG_INFO("mu: " << mu_);
}
