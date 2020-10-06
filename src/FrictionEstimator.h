#pragma once

#include <mc_rbdyn/Robot.h>
#include <mc_rtc/Configuration.h>
#include <Eigen/Dense>

struct FrictionEstimator
{
 public:
  FrictionEstimator(const mc_rbdyn::Robot& robot, const std::string & surfaceName, const Eigen::Vector3d & surfaceNormal, const double & initialFrictionGuess);
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
  double mu_filtered() const
  {
    return mu_filtered_;
  }
  double forceX() const
  {
    return force_x_;
  }
  double forceY() const
  {
    return force_y_;
  }
  double forceZ() const
  {
    return force_z_;
  }

  Eigen::Matrix6d RotationMat;

 protected:
  std::string surfaceName_;
  double initialFrictionGuess_;
  std::string Name_fs;
  Eigen::Vector3d surfaceNormal_;
  std::string rightHandForceSensor = "RightHandForceSensor";
  std::string leftHandForceSensor = "RightHandForceSensor";
  std::string rightFootForceSensor = "RightFootForceSensor";
  std::string leftFootForceSensor = "LeftFootForceSensor";
  sva::ForceVecd wrench_fs; 
  Eigen::Vector3d wrench_fs_local;
  sva::MotionVecd bodyVel;
  Eigen::Vector3d VelocityVec, localVel;
  
  double force_x_, force_y_, force_z_;
  double mu_x_, mu_y_, mu_;
  double mu_calc_;
  double mu_filtered_;
  double alpha;
};
