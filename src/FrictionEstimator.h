#pragma once

#include <mc_rbdyn/Robot.h>

struct FrictionEstimator
{
 public:
  FrictionEstimator(const mc_rbdyn::Robot& robot, const std::string & surfaceName, const Eigen::Vector3d & surfaceNormal);
  void update(const mc_rbdyn::Robot& robot);

  double mu_x() const
  {
    return mu_x_;
  }
  double mu_y() const
  {
    return mu_y_;
  }
  double mu() const
  {
    return mu_;
  }
  double mu_calc() const
  {
    return mu_calc_;
  }

  Eigen::Matrix6d RotationMat;

 protected:
  std::string surfaceName_;
  Eigen::Vector3d surfaceNormal_;
  std::string rightHandForceSensor = "RightHandForceSensor";
  std::string leftHandForceSensor = "RightHandForceSensor";
  std::string rightFootForceSensor = "RightFootForceSensor";
  std::string leftFootForceSensor = "LeftFootForceSensor";
  sva::ForceVecd wrench_fs; 
  Eigen::Vector3d wrench_fs_rot;
  sva::MotionVecd bodyVel;
  Eigen::Vector3d VelocityVec, localVel;

  double mu_x_, mu_y_, mu_, mu_calc_;
};
