#include "FrictionEstimator.h"
#include <mc_rtc/logging.h>

FrictionEstimator::FrictionEstimator(const mc_rbdyn::Robot& robot, const std::string & surfaceName, const Eigen::Vector3d & surfaceNormal, const double & initialFrictionGuess)
  : surfaceName_(surfaceName),
    surfaceNormal_(surfaceNormal),
    initialFrictionGuess_(initialFrictionGuess)
{
  RotationMat.setZero();
  mu_calc_ = initialFrictionGuess_;
  mu_filtered_ = initialFrictionGuess_;
  alpha = 0.999;
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

  Eigen::Matrix3d constRot; 
  if(surfaceName_ == "RightHandPad"){
    constRot << 0.0,  0.0, -1.0,
                1.0,  0.0,  0.0,
                0.0, -1.0,  0.0;
    Name_fs = rightHandForceSensor;
  }
  else if(surfaceName_ == "LeftHandPad"){
    constRot <<  0.0,  0.0, -1.0,
                 -1.0,  0.0,  0.0,
                  0.0,  1.0,  0.0;
    Name_fs = leftHandForceSensor;
  }
  else if (surfaceName_ == "BlockLeftHand"){
    constRot <<  1.0,  0.0,  0.0,
                 0.0,  1.0,  0.0,
                 0.0,  0.0,  1.0;
    Name_fs = leftHandForceSensor;
  }
  else if (surfaceName_ == "LeftFoot"){
    constRot <<  1.0,  0.0,  0.0,
                 0.0,  1.0,  0.0,
                 0.0,  0.0,  1.0;
    Name_fs = leftFootForceSensor;
  }
  else if (surfaceName_ == "RightFoot"){
    constRot <<  1.0,  0.0,  0.0,
                 0.0,  1.0,  0.0,
                 0.0,  0.0,  1.0;
    Name_fs = rightFootForceSensor;
  }

  const auto bodyRot = constRot * bodyPT.rotation();

  RotationMat.block<3, 3>(0, 0) = bodyRot;
  RotationMat.block<3, 3>(3, 3) = bodyRot;
  wrench_fs = robot.forceSensor(Name_fs).worldWrench(robot);
  wrench_fs_local << wrench_fs.force().x(), wrench_fs.force().y(), wrench_fs.force().z();
  wrench_fs_local = bodyRot * wrench_fs_local;
  bodyVel = robot.bodyVelW(bodyName);
  VelocityVec << bodyVel.linear().x(), bodyVel.linear().y(), bodyVel.linear().z(); //Global = [0, 0, vel]
  localVel = bodyRot * VelocityVec; 
  double velNorm = Eigen::Vector3d{localVel(0), localVel(1), 0.}.norm();
  Eigen::Vector3d velRatio_ = localVel / velNorm;
  //mu_calc_ = - wrench_fs_local(0) / (wrench_fs_local(2) * velRatio_(0));
  
  if(abs(wrench_fs_local(2)) > 0.5){
    mu_calc_ = (sqrt( std::pow(wrench_fs_local(0),2) + std::pow(wrench_fs_local(1),2) )) / abs(wrench_fs_local(2));
  }
  mu_filtered_ = alpha * mu_filtered_ + (1 - alpha) * mu_calc_;
}
