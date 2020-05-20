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

 protected:
  std::string surfaceName_;
  Eigen::Vector3d surfaceNormal_;

  double mu_x_, mu_y_, mu_;
};
