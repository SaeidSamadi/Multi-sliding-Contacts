#include "FrictionEstimator.h"
#include <mc_rtc/logging.h>

FrictionEstimator::FrictionEstimator(const mc_rbdyn::Robot& robot, const std::string & surfaceName, const Eigen::Vector3d & surfaceNormal)
  : surfaceName_(surfaceName),
    surfaceNormal_(surfaceNormal)
{
  RotationMat.setZero();
}

void FrictionEstimator::update(const mc_rbdyn::Robot& robot)
{
  const auto & surface = robot.surface(surfaceName_);
  const auto & wrench = robot.surfaceWrench(surfaceName_);
  const auto & bodyName = surface.bodyName();
  //const auto & bodyVel = robot.bodyVelB(bodyName);
  //const auto & surfaceVel = surface.X_b_s() * bodyVel;
  //Eigen::Vector3d velDir = surfaceVel.linear().normalized();
  force_x_ = wrench.force().x();
  force_y_ = wrench.force().y();
  force_z_ = wrench.force().z();
  const auto bodyPT = robot.bodyPosW(bodyName);

  //XXX To add constRots of different surfaces
  Eigen::Matrix3d constRot; 
  constRot << 0.0,  0.0, -1.0,
              1.0,  0.0,  0.0,
              0.0, -1.0,  0.0;
  std::string Name_fs = rightHandForceSensor; // XXX if/else
  //std::string Name_fs = rightHandForceSensor; // XXX if/else

  const auto bodyRot = constRot * bodyPT.rotation();

  RotationMat.block<3, 3>(0, 0) = bodyRot;
  RotationMat.block<3, 3>(3, 3) = bodyRot;
  wrench_fs = robot.forceSensor(Name_fs).worldWrench(robot);
  wrench_fs_rot << wrench_fs.force().x(), wrench_fs.force().y(), wrench_fs.force().z();
  wrench_fs_rot = bodyRot * wrench_fs_rot;
  bodyVel = robot.bodyVelW(bodyName);
  VelocityVec << bodyVel.linear().x(), bodyVel.linear().y(), bodyVel.linear().z(); //Global = [0, 0, vel]
  localVel = bodyRot * VelocityVec; 
  double velNorm = Eigen::Vector3d{localVel(0), localVel(1), 0.}.norm();
  Eigen::Vector3d velRatio_ = localVel / velNorm;
  mu_calc_ = - wrench_fs_rot(0) / (wrench_fs_rot(2) * velRatio_(0));





  // OLD but gold :))

  ////mu_x_ = 0.;
  ////double N =   fabs(wrench.force().z());
  ////double fx =  fabs(wrench.force().x());
  ////double vxy = Eigen::Vector2d{ surfaceVel.linear().x(), surfaceVel.linear().y() }.norm();
  ////if(fx > 10e-8)
  ////{
  ////  double vx =  fabs(surfaceVel.linear().x());
  ////  mu_x_ = fx/N * vxy / vx;
  ////}

  ////mu_y_ = 0;
  ////double vy =  fabs(surfaceVel.linear().y());
  ////if(vy > 10e-8)
  ////{
  ////  double fy =  fabs(wrench.force().y());
  ////  mu_y_ = fy/N * vxy / vy;
  ////}

  ////mu_ = 0.;
  ////if(mu_x_ < 10e-8)
  ////{
  ////  mu_ = mu_y_;
  ////}
  ////else if(mu_y_ < 10e-8)
  ////{
  ////  mu_ = mu_x_;
  ////}
  ////else
  ////{
  //// mu_ = (mu_x_ + mu_y_)/2.;
  ////}
  ////// LOG_INFO("mu_x: " << mu_x_);
  ////// LOG_INFO("mu_y: " << mu_y_);
  ////// LOG_INFO("mu: " << mu_);
}
