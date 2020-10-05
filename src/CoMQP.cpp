#include "CoMQP.h"

#include <mc_rtc/logging.h>
using namespace std;

CoMQP::CoMQP(const mc_rbdyn::Robot & robot, const mc_rtc::Configuration & config)
{

  //numVar_ = 3 + 6 * 3 + 1;
  numVar_ = 3 + 6 * 4 + 1;
  //updateNumVar(robot);

// Offset mass
  mg = (massOffset_ + robot.mass()) * robot.mbc().gravity.z();

  debug_ = config("debug", false);
  X_lf = config("left_foot")("shape")[0];
  Y_lf = config("left_foot")("shape")[1];
  X_rf = config("right_foot")("shape")[0];
  Y_rf = config("right_foot")("shape")[1];
  X_rh = config("right_hand")("shape")[0];
  Y_rh = config("right_hand")("shape")[1];
  mu_rh = config("right_hand")("friction");
  mu_lh = config("right_hand")("friction");
  mu_rf = config("right_foot")("friction");
  mu_lf = config("left_foot")("friction");
  com_pos = config("com")("pos");

  E_m1.resize(6, 3);
  E_m1.setZero();
  E_m1(3, 1) = -mg;
  E_m1(4, 0) = mg;

  E_m2.resize(6);
  E_m2.setZero();
  E_m2(2) = mg;

  E_rf.resize(6, 6);
  E_rf.setZero();
  E_lf.resize(6, 6);
  E_lf.setZero();
  E_rh.resize(6, 6);
  E_rh.setZero();
  E_lh.resize(6, 6);
  E_lh.setZero();

  
  sliding_rh1.resize(6, 6);
  sliding_rh1.setZero();
  sliding_rh1(0, 0) = 1.0;
  sliding_rh1(1, 1) = 1.0;
  sliding_rh1(2, 2) = 1.0;
  sliding_rh2.resize(6, 6);
  sliding_rh2.setZero();

  sliding_rh.resize(6, 6);
  sliding_rh.setZero();

  sliding_lh1.resize(6, 6);
  sliding_lh1.setZero();
  sliding_lh1(0, 0) = 1.0;
  sliding_lh1(1, 1) = 1.0;
  sliding_lh1(2, 2) = 1.0;
  sliding_lh2.resize(6, 6);
  sliding_lh2.setZero();

  sliding_lh.resize(6, 6);
  sliding_lh.setZero();

  A.resize(24, 27);
  A.setZero();

  A_st.resize(24, 28);
  A_st.setZero();

  b.resize(24);
  b.setZero();

  Ineq_mat1.resize(6, 27); //For 3contacts: (6, 21)
  Ineq_mat2.resize(6, 27);
  Ineq_mat3.resize(6, 27);
  Ineq_mat4.resize(6, 27);
  Ineq_mat5.resize(6, 27);
  Ineq_mat6.resize(6, 27);
  Ineq_mat7.resize(6, 27);
  Ineq_mat8.resize(6, 27);
  Ineq_mat1.setZero();
  Ineq_mat2.setZero();
  Ineq_mat3.setZero();
  Ineq_mat4.setZero();
  Ineq_mat5.setZero();
  Ineq_mat6.setZero();
  Ineq_mat7.setZero();
  Ineq_mat8.setZero();

  Ineq_max_rf.resize(4, 27);
  Ineq_min_rf.resize(4, 27);
  Ineq_max_lf.resize(4, 27);
  Ineq_min_lf.resize(4, 27);
  Ineq_max_rh.resize(4, 27);
  Ineq_min_rh.resize(4, 27);
  Ineq_max_lh.resize(4, 27);
  Ineq_min_lh.resize(4, 27);
  Ineq_max_rf.setZero();
  Ineq_min_rf.setZero();
  Ineq_max_lf.setZero();
  Ineq_min_lf.setZero();
  Ineq_max_rh.setZero();
  Ineq_min_rh.setZero();
  Ineq_max_lh.setZero();
  Ineq_min_lh.setZero();

  G.resize(80, 27);
  G.setZero();

  G_st.resize(81, 28);
  G_st.setZero();

  h.resize(80);
  h.setZero();

  h_st.resize(81);
  h_st.setZero();

  P.resize(28, 28);
  P.setZero();


  Y_desired.resize(28);
  Y_desired.setZero();

  q.resize(28);
  q.setZero();


  solver_.problem(nrVar(), nrEq(), nrInEq());
}

//int CoMQP::updateNumVar(const mc_rbdyn::Robot & robot){
//  double NormalForce_rh = robot.surfaceWrench(rightHandSurface).force().z();
//  double NormalForce_rh_abs = fabs(NormalForce_rh);
//  double err = fabs(NormalForce_rh - tmp);
//  //mc_rtc::log::info(NormalForce_rh_abs);
//  //mc_rtc::log::info(tmp);
//  tmp = NormalForce_rh;
//  if(NormalForce_rh_abs > 8.0 && err < 0.3){
//    mc_rtc::log::info("Hand Contact added ***************** ");
//    numVar_ = 22;
//  }
//  else{
//    numVar_ = 22;
//    mc_rtc::log::info("else");
//  }
//}

bool CoMQP::solve(const mc_rbdyn::Robot & robot)
{
  updateContactPoses(robot);

  const auto bodyName_rf = robot.surface(rightFootSurface).bodyName();
  const auto bodyName_lf = robot.surface(leftFootSurface).bodyName();
  const auto bodyName_rh = robot.surface(rightHandSurface).bodyName();
  const auto bodyName_lh = robot.surface(leftHandSurface).bodyName();

  const auto bodyPT_rf = robot.bodyPosW(bodyName_rf);
  const auto bodyPT_lf = robot.bodyPosW(bodyName_lf);
  const auto bodyPT_rh = robot.bodyPosW(bodyName_rh);
  const auto bodyPT_lh = robot.bodyPosW(bodyName_lh);
 
  const double Px_rh = rhPose.translation().x();
  const double Py_rh = rhPose.translation().y();
  const double Pz_rh = rhPose.translation().z();

  const double Px_lh = lhPose.translation().x();
  const double Py_lh = lhPose.translation().y();
  const double Pz_lh = lhPose.translation().z();
    
  const double Px_rh_rot = rhPose.rotation()(0,0);
  const double Py_rh_rot = rhPose.rotation()(1,1);
  const double Pz_rh_rot = rhPose.rotation()(2,2);
  
  E_rf = graspMatrixFromTranslation(rfPose.translation());
  E_rh = graspMatrixFromTranslation(rhPose.translation());
  E_lf = graspMatrixFromTranslation(lfPose.translation());
  E_lh = graspMatrixFromTranslation(lhPose.translation());
  
  //rightHandPose_x = Px_rh_rot;
  //rightHandPose_y = Py_rh_rot;
  //rightHandPose_z = Pz_rh_rot;
  rightHandPose(0) = Px_rh_rot;
  rightHandPose(1) = Py_rh_rot;
  rightHandPose(2) = Pz_rh_rot;
  rightHandPose(3) = Px_rh;
  rightHandPose(4) = Py_rh;
  rightHandPose(5) = Pz_rh;
  
  Eigen::Matrix3d constRot_rf; // Rotation from body local link to desired surface orien.
  Eigen::Matrix3d constRot_lf;
  Eigen::Matrix3d constRot_rh;
  Eigen::Matrix3d constRot_lh;
  constRot_rf = Eigen::Matrix3d::Identity(); // Foot Surface is aligned with body link axis
  constRot_lf = constRot_rf;
  constRot_rh << 0.0,  0.0, -1.0,
                 1.0,  0.0,  0.0,
                 0.0, -1.0,  0.0;
  // constRot_lh <<  0.0,  0.0, -1.0,
  //                -1.0,  0.0,  0.0,
  //                 0.0,  1.0,  0.0;
  constRot_lh <<  1.0,  0.0,  0.0,
                  0.0,  1.0,  0.0,
                  0.0,  0.0,  1.0;
  //constRot_lh = constRot_rh;

  const auto bodyRot_rf = constRot_rf * bodyPT_rf.rotation();
  const auto bodyRot_lf = constRot_lf * bodyPT_lf.rotation();
  const auto bodyRot_rh = constRot_rh * bodyPT_rh.rotation();
  const auto bodyRot_lh = constRot_lh * bodyPT_lh.rotation();

  Eigen::Matrix6d Rot_rf;
  Rot_rf.setZero();
  Eigen::Matrix6d Rot_lf;
  Rot_lf.setZero();
  Eigen::Matrix6d Rot_rh;
  Rot_rh.setZero();
  Eigen::Matrix6d Rot_lh;
  Rot_lh.setZero();

  Rot_rf.block<3, 3>(0, 0) = bodyRot_rf;
  Rot_rf.block<3, 3>(3, 3) = bodyRot_rf;
  Rot_lf.block<3, 3>(0, 0) = bodyRot_lf;
  Rot_lf.block<3, 3>(3, 3) = bodyRot_lf;
  Rot_rh.block<3, 3>(0, 0) = bodyRot_rh;
  Rot_rh.block<3, 3>(3, 3) = bodyRot_rh;
  Rot_lh.block<3, 3>(0, 0) = bodyRot_lh;
  Rot_lh.block<3, 3>(3, 3) = bodyRot_lh;
  
  
  // XXX measure mu_y and mu_z from force sensor here.
  // Usually Saeid sets it according to the trajectory
  // Let's see what's better
  // {{{
  // const auto & rh_fs = robot.forceSensor(rightHandForceSensor).worldWrenchWithoutGravity(robot);
  // Eigen::Vector3d mu = rh_fs.force()/N;
  // mu_y = mu.y();
  // mu_z = mu.z();
  // }}}
  rh_fs = robot.forceSensor(rightHandForceSensor).worldWrench(robot);
  rh_fs_rot << rh_fs.force().x(), rh_fs.force().y(), rh_fs.force().z();
  rh_fs_rot = bodyRot_rh * rh_fs_rot;
  rhVel = robot.bodyVelW("r_wrist");
  lhVel = robot.bodyVelW("l_wrist");
  RWristVel << rhVel.linear().x(), rhVel.linear().y(), rhVel.linear().z(); //Global = [0, 0, vel]
  LWristVel << lhVel.linear().x(), lhVel.linear().y(), lhVel.linear().z(); //Global = [0, 0, vel]
  localVel_rh = bodyRot_rh * RWristVel;
  localVel_lh = bodyRot_lh * LWristVel;
  double velNorm_rh = Eigen::Vector3d{localVel_rh(0), localVel_rh(1), 0.}.norm();
  double velNorm_lh = Eigen::Vector3d{localVel_lh(0), localVel_lh(1), 0.}.norm();
  Eigen::Vector3d velRatio_ = - localVel_rh / velNorm_rh;

  if(velNorm_rh < 10e-5)
  {
    mu_x_rh = mu_rh/2.;
    mu_y_rh = mu_rh/2.;
  }
  else
  {
    //mu_x_rh = mu_rh * fabs(rhVel.linear().x()) / velNorm_rh;
    //mu_y_rh = mu_rh * fabs(rhVel.linear().y()) / velNorm_rh;
    mu_x_rh = mu_rh * fabs(localVel_rh(0)) / velNorm_rh;
    mu_y_rh = mu_rh * fabs(localVel_rh(1)) / velNorm_rh;
  }
  
  if( fabs(rh_fs_rot(2)) > 5.0 && //f_z > 1.0
      fabs(rh_fs_rot(0)) / fabs(rh_fs_rot(2)) >= 0.1 && //0.1 < mu < 3.0
      fabs(rh_fs_rot(0)) / fabs(rh_fs_rot(2)) <= 3.0 ){
  mu_x_calc = rh_fs_rot(0) / (rh_fs_rot(2) * velRatio_(0));
  mu_x_avg = mu_x_calc;
  }
  else{
    mu_x_calc = mu_x_avg;
  }

  muN = mu_x_calc * rh_fs_rot(2);

  sliding_rh2(0, 2) = mu_x_rh;
  sliding_rh2(1, 2) = mu_y_rh;
  sliding_rh = sliding_rh1 - sliding_rh2;
  sliding_rh = sliding_rh * Rot_rh;

  
  if(velNorm_lh < 10e-5)
  {
    mu_x_lh = mu_lh/2.;
    mu_y_lh = mu_lh/2.;
  }
  else
  {
    mu_x_lh = mu_lh * fabs(localVel_lh(0)) / velNorm_lh;
    mu_y_lh = mu_lh * fabs(localVel_lh(1)) / velNorm_lh;
  }

  sliding_lh2(0, 2)= mu_x_lh;
  sliding_lh2(1, 2)= mu_y_lh;
  sliding_lh = sliding_lh1 - sliding_lh2;
  sliding_lh = sliding_lh * Rot_lh;
  Eigen::Matrix6d Mat_lf;
  Mat_lf.setZero();
  Mat_lf(2, 2) = 1.0;

  A_st.block<6, 3>(0, 0) = E_m1;
  A_st.block<6, 6>(0, 3) = E_rf;
  A_st.block<6, 6>(0, 9) = E_lf;
  A_st.block<6, 6>(0, 15) = E_rh;
  A_st.block<6, 6>(0, 21) = E_lh;
  A_st.block<6, 6>(6, 15) = sliding_rh;
  A_st.block<6, 6>(12, 21) = sliding_lh;
  A_st.block<6, 6>(18, 9) = Mat_lf;

  b.head<6>() = E_m2.head<6>();
  b(8) = -N_rh;
  b(14) = -N_lh;
  b(20) = N_lf;

  Eigen::Matrix6d Ineq_frictionCone_rf;
  Ineq_frictionCone_rf.setZero();
  Ineq_frictionCone_rf(0, 2) = mu_rf;
  Ineq_frictionCone_rf(1, 2) = mu_rf;
  Ineq_frictionCone_rf(3, 2) = Y_rf;
  Ineq_frictionCone_rf(4, 2) = X_rf;

  Eigen::Matrix6d Ineq_frictionCone_lf;
  Ineq_frictionCone_lf.setZero();
  Ineq_frictionCone_lf(0, 2) = mu_lf;
  Ineq_frictionCone_lf(1, 2) = mu_lf;
  Ineq_frictionCone_lf(3, 2) = Y_lf;
  Ineq_frictionCone_lf(4, 2) = X_lf;

  Eigen::Matrix6d Ineq_frictionCone_rh;
  Ineq_frictionCone_rh.setZero();
  Ineq_frictionCone_rh(0, 2) = mu_rh;
  Ineq_frictionCone_rh(1, 2) = mu_rh;
  Ineq_frictionCone_rh(3, 2) = Y_rh;
  Ineq_frictionCone_rh(4, 2) = X_rh;

  Eigen::Matrix6d Ineq_frictionCone_lh;
  Ineq_frictionCone_lh.setZero();
  Ineq_frictionCone_lh(0, 2) = mu_lh;
  Ineq_frictionCone_lh(1, 2) = mu_lh;
  Ineq_frictionCone_lh(3, 2) =  Y_lh;
  Ineq_frictionCone_lh(4, 2) =  X_lh;

  UBmat_ineq_rf = Eigen::Matrix6d::Identity() - Ineq_frictionCone_rf;
  UBmat_ineq_rf(5, 5) = 0.0;
  LBmat_ineq_rf = -Eigen::Matrix6d::Identity() - Ineq_frictionCone_rf;
  LBmat_ineq_rf(5, 5) = 0.0;
  UBmat_ineq_rf = UBmat_ineq_rf * Rot_rf;
  LBmat_ineq_rf = LBmat_ineq_rf * Rot_rf;

  UBmat_ineq_lf = Eigen::Matrix6d::Identity() - Ineq_frictionCone_lf;
  UBmat_ineq_lf(5, 5) = 0.0;
  LBmat_ineq_lf = -Eigen::Matrix6d::Identity() - Ineq_frictionCone_lf;
  LBmat_ineq_lf(5, 5) = 0.0;
  UBmat_ineq_lf = UBmat_ineq_lf * Rot_lf;
  LBmat_ineq_lf = LBmat_ineq_lf * Rot_lf;

  UBmat_ineq_rh = Eigen::Matrix6d::Identity() - Ineq_frictionCone_rh;
  UBmat_ineq_rh(5, 5) = 0.0;
  LBmat_ineq_rh = -Eigen::Matrix6d::Identity() - Ineq_frictionCone_rh;
  LBmat_ineq_rh(5, 5) = 0.0;

  UBmat_ineq_lh = Eigen::Matrix6d::Identity() - Ineq_frictionCone_lh;
  UBmat_ineq_lh(5, 5) = 0.0;
  LBmat_ineq_lh = -Eigen::Matrix6d::Identity() - Ineq_frictionCone_lh;
  LBmat_ineq_lh(5, 5) = 0.0;

  UBmat_ineq_rh.block<3, 3>(0, 0) = Eigen::Matrix3d::Zero(); //needed for sliding contacts - For fixed: Do not setZero()
  UBmat_ineq_rh.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
  UBmat_ineq_rh = UBmat_ineq_rh * Rot_rh;

  LBmat_ineq_rh.block<3, 3>(0, 0) = Eigen::Matrix3d::Zero();
  LBmat_ineq_rh.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
  LBmat_ineq_rh = LBmat_ineq_rh * Rot_rh;



  Ineq_mat1.block<6, 6>(0, 3) = UBmat_ineq_rf;
  Ineq_mat2.block<6, 6>(0, 3) = LBmat_ineq_rf;
  Ineq_mat3.block<6, 6>(0, 9) = UBmat_ineq_lf;
  Ineq_mat4.block<6, 6>(0, 9) = LBmat_ineq_lf;
  Ineq_mat5.block<6, 6>(0, 15) = UBmat_ineq_rh; // So for Sliding contacts: Zero() matrix
  Ineq_mat6.block<6, 6>(0, 15) = LBmat_ineq_rh;
  Ineq_mat7.block<6, 6>(0, 21) = UBmat_ineq_lh;
  Ineq_mat8.block<6, 6>(0, 21) = LBmat_ineq_lh;

  double coef_rf = -mu_rf * (X_rf + Y_rf);
  double coef_lf = -mu_lf * (X_lf + Y_lf);
  double coef_rh = -mu_rh * (X_rh + Y_rh);
  double coef_lh = -mu_lh * (X_lh + Y_lh);
  Eigen::MatrixXd tauZ_max_rf(4, 6), tauZ_min_rf(4, 6);
  Eigen::MatrixXd tauZ_max_lf(4, 6), tauZ_min_lf(4, 6);
  Eigen::MatrixXd tauZ_max_rh(4, 6), tauZ_min_rh(4, 6);
  Eigen::MatrixXd tauZ_max_lh(4, 6), tauZ_min_lh(4, 6);
  tauZ_max_rf <<
           Y_rf,   X_rf, coef_rf,  mu_rf,  mu_rf, 1.,
           -Y_rf,  X_rf, coef_rf, -mu_rf,  mu_rf, 1.,
           Y_rf,  -X_rf, coef_rf,  mu_rf, -mu_rf, 1.,
           -Y_rf, -X_rf, coef_rf, -mu_rf, -mu_rf, 1.;
  tauZ_min_rf <<
           Y_rf, X_rf, coef_rf, -mu_rf, -mu_rf, -1.,
           -Y_rf, X_rf, coef_rf, mu_rf, -mu_rf, -1.,
           Y_rf, -X_rf, coef_rf, -mu_rf, mu_rf, -1.,
           -Y_rf, -X_rf, coef_rf, mu_rf, mu_rf, -1.;
  tauZ_max_lf <<
           Y_lf,  X_lf,  coef_lf,  mu_lf,  mu_lf, 1.,
           -Y_lf, X_lf,  coef_lf, -mu_lf,  mu_lf, 1.,
           Y_lf,  -X_lf, coef_lf,  mu_lf, -mu_lf, 1.,
           -Y_lf, -X_lf, coef_lf, -mu_lf, -mu_lf, 1.;
  tauZ_min_lf <<
            Y_lf, X_lf, coef_lf, -mu_lf, -mu_lf, -1.,
            -Y_lf, X_lf, coef_lf, mu_lf, -mu_lf, -1.,
            Y_lf, -X_lf, coef_lf, -mu_lf, mu_lf, -1.,
            -Y_lf, -X_lf, coef_lf, mu_lf, mu_lf, -1.;
  tauZ_max_rh <<
            Y_rh,  X_rh, coef_rh,  mu_rh,  mu_rh, 1.,
           -Y_rh,  X_rh, coef_rh, -mu_rh,  mu_rh, 1.,
            Y_rh, -X_rh, coef_rh,  mu_rh, -mu_rh, 1.,
           -Y_rh, -X_rh, coef_rh, -mu_rh, -mu_rh, 1.;
  tauZ_min_rh <<
             Y_rh,  X_rh, coef_rh, -mu_rh, -mu_rh, -1.,
            -Y_rh,  X_rh, coef_rh,  mu_rh, -mu_rh, -1.,
             Y_rh, -X_rh, coef_rh, -mu_rh,  mu_rh, -1.,
            -Y_rh, -X_rh, coef_rh,  mu_rh,  mu_rh, -1.;
  tauZ_max_lh <<
            Y_lh,  X_lh, coef_lh,  mu_lh,  mu_lh, 1.,
           -Y_lh,  X_lh, coef_lh, -mu_lh,  mu_lh, 1.,
            Y_lh, -X_lh, coef_lh,  mu_lh, -mu_lh, 1.,
           -Y_lh, -X_lh, coef_lh, -mu_lh, -mu_lh, 1.;
  tauZ_min_lh <<
            Y_lh,  X_lh, coef_lh, -mu_lh, -mu_lh, -1.,
           -Y_lh,  X_lh, coef_lh,  mu_lh, -mu_lh, -1.,
            Y_lh, -X_lh, coef_lh, -mu_lh,  mu_lh, -1.,
           -Y_lh, -X_lh, coef_lh,  mu_lh,  mu_lh, -1.;
  // clang-format off
  Ineq_max_rf.block<4,6>(0,3) = tauZ_max_rf * Rot_rf;
  Ineq_min_rf.block<4,6>(0,3) = tauZ_min_rf * Rot_rf;
  Ineq_max_lf.block<4,6>(0,9) = tauZ_max_lf * Rot_lf;
  Ineq_min_lf.block<4,6>(0,9) = tauZ_min_lf * Rot_lf;
  Ineq_max_rh.block<4,6>(0,15) = tauZ_min_rh * Rot_rh; //XXX min is used for max!!!
  Ineq_min_rh.block<4,6>(0,15) = tauZ_max_rh * Rot_rh;
  Ineq_max_lh.block<4,6>(0,21) = tauZ_min_lh * Rot_lh; //XXX min is used for max!!!
  Ineq_min_lh.block<4,6>(0,21) = tauZ_max_lh * Rot_lh;
  // clang-format on

  G.block<6, 21>(0, 0) = Ineq_mat1;
  G.block<6, 21>(6, 0) = Ineq_mat2;
  G.block<6, 21>(12, 0) = Ineq_mat3;
  G.block<6, 21>(18, 0) = Ineq_mat4;
  G.block<6, 21>(24, 0) = Ineq_mat5;
  G.block<6, 21>(30, 0) = Ineq_mat6;
  G.block<4, 21>(36, 0) = Ineq_max_rf;
  G.block<4, 21>(40, 0) = Ineq_min_rf;
  G.block<4, 21>(44, 0) = Ineq_max_lf;
  G.block<4, 21>(48, 0) = Ineq_min_lf;
  G.block<4, 21>(52, 0) = Ineq_max_rh;
  G.block<4, 21>(56, 0) = Ineq_min_rh;
  G.block<6, 21>(62, 0) = Ineq_mat7;
  G.block<6, 21>(68, 0) = Ineq_mat8;
  G.block<4, 21>(72, 0) = Ineq_max_lh;
  G.block<4, 21>(76, 0) = Ineq_min_lh;
  G_st.block<80, 27>(0, 0) = G;

  for (int i=0; i<20*CoMQP::k; i++){
  double zeta = G.block<1,27>(i,0).norm();
  G_st(i,3+6*k) = zeta;
  //cout<<G_st(i,3+6*k)<<endl;
  };
  G_st(20*k, 3+6*k) = -1.;

  h_st(2) = fz_max; //For Fixed contacts set it for normal component; Horizental: f_z, Vertical Fix: f_x
  h_st(8) = -fz_min;
  h_st(14) = fz_max;
  h_st(20) = -fz_min;

  P.block<3, 3>(0, 0) = P_PG * Eigen::Matrix3d::Identity();
  P.block<3, 3>(3, 3) = P_Force_rf * Eigen::Matrix3d::Identity();
  P.block<3, 3>(6, 6) = P_Wrench_rf * Eigen::Matrix3d::Identity();
  P.block<3, 3>(9, 9) = P_Force_lf * Eigen::Matrix3d::Identity();
  P.block<3, 3>(12, 12) = P_Wrench_lf * Eigen::Matrix3d::Identity();
  P.block<3, 3>(15, 15) = P_Force_rh * Eigen::Matrix3d::Identity();
  P.block<3, 3>(18, 18) = P_Wrench_rh * Eigen::Matrix3d::Identity();
  P.block<3, 3>(21, 21) = P_Force_lh * Eigen::Matrix3d::Identity();  // size P: 22x22 -> 28x28
  P.block<3, 3>(24, 24) = P_Wrench_lh * Eigen::Matrix3d::Identity(); // Also set nVar_
  P(27,27) = P_Radius; //Put it at last
  //P.Identity(22,22);
  P = 2 * P;

  //Eigen::Vector3d PG_d{com_x_d, com_y_d, com_pos.z()};
  Eigen::Vector3d PG_d{com_pos.x(), com_pos.y(), com_pos.z()};
  //LOG_INFO("desired_CoM ="<<PG_d);
  //CoMQP::PG_d.setZero();
  //Eigen::Vector3d f_rf_d{-N / 2, -(mu_y * N) / 2, (mg - mu_z * N) / 2};
  //Eigen::Vector3d f_rf_d;
  //f_rf_d.setZero();
  //Eigen::Vector3d tau_rf_d = Eigen::Vector3d{Px_rf, Py_rf, Pz_rf}.cross(f_rf_d);
  //Eigen::Vector3d f_lf_d = f_rf_d;
  //Eigen::Vector3d tau_lf_d = Eigen::Vector3d{Px_lf, Py_lf, Pz_lf}.cross(f_lf_d);
  Eigen::Vector3d f_rh_d{N_rh, mu_x_rh * N_rh, mu_y_rh * N_rh}; //XXX Set sliding Condition here
  Eigen::Vector3d tau_rh_d = Eigen::Vector3d{Px_rh, Py_rh, Pz_rh}.cross(f_rh_d);
  Eigen::Vector3d f_lh_d{N_lh, mu_x_lh * N_lh, mu_y_lh * N_lh}; //XXX Set sliding Condition here
  Eigen::Vector3d tau_lh_d = Eigen::Vector3d{Px_lh, Py_lh, Pz_lh}.cross(f_lh_d);

  //Y_desired.block<3, 1>(0, 0) = PG_d;
  //Y_desired.block<3, 1>(3, 0) = f_rf_d;
  //Y_desired.block<3, 1>(6, 0) = tau_rf_d;
  //Y_desired.block<3, 1>(9, 0) = f_lf_d;
  //Y_desired.block<3, 1>(12, 0) = tau_lf_d;
  Y_desired.block<3, 1>(15, 0) = f_rh_d;     // uncomment or add for sliding contact
  Y_desired.block<3, 1>(18, 0) = tau_rh_d;   // q_size
  Y_desired.block<3, 1>(21, 0) = f_lh_d;     // uncomment or add for sliding contact
  Y_desired.block<3, 1>(24, 0) = tau_lh_d;   // q_size
  q = -2* Y_desired;
  q(27) = 1.;


  /*
  cout<<"A_st size" << A_st.rows()<< "x" << A_st.cols() << endl;
  cout<<"b size" << b.rows()<< "x" << b.cols() << endl;
  cout<<"G_st size" << G_st.rows()<< "x" << G_st.cols() << endl;
  cout<<"h_st size" << h_st.rows()<< "x" << h_st.cols() << endl;
  */


  if(debug_)
  {
    mc_rtc::log::info("mu_y: {}", mu_x_lh);
    mc_rtc::log::info("mu_z: {}", mu_y_rh);
    mc_rtc::log::info("P:\n{}", P);
    mc_rtc::log::info("\n\nq: {}", q.transpose());
    mc_rtc::log::info("\n\nA:\n{}", A);
    mc_rtc::log::info("\n\nb: {}", b.transpose());
    mc_rtc::log::info("\n\nG:\n{}", G);
    mc_rtc::log::info("\n\nh: {}", h.transpose());
  }

  bool ret = solver_.solve(P, q, A_st, b, G_st, h_st);
  resetContactPoses();  
  return ret;
}

const Eigen::VectorXd & CoMQP::resultVector() const
{
  return solver_.result();
}

CoMQPResult CoMQP::result() const
{
  const Eigen::VectorXd & r = resultVector();
  CoMQPResult res;
  res.comPos = r.head<3>();
  //Eigen::Vector3d CoMQP::PG_d = res.comPos;
  res.rightFootForce = sva::ForceVecd(r.segment(6, 3), r.segment(3, 3));
  res.leftFootForce = sva::ForceVecd(r.segment(12, 3), r.segment(9, 3));
  res.rightHandForce = sva::ForceVecd(r.segment(18, 3), r.segment(15, 3));
  res.leftHandForce = sva::ForceVecd(r.segment(24, 3), r.segment(21, 3));
  //LOG_SUCCESS("info : " << r.transpose() << " foobar")
  //cout<<r(21)<<endl;
  //cout<<r.size()<<endl;
  return res;
}

int CoMQP::errorCode() const
{
  return solver_.fail();
}

void CoMQP::errorMessage() const
{
  std::ostringstream stream;
  //solver_.inform(stream);
  mc_rtc::log::error(stream.str());
}

void CoMQP::resetContactPoses()
{
  lfPoseUpdated = false;
  rfPoseUpdated = false;
  lhPoseUpdated = false;
  rhPoseUpdated = false;
}

void CoMQP::updateContactPoses(const mc_rbdyn::Robot & robot)
{
  updateRFPose(robot.surface(rightFootSurface).X_0_s(robot));
  updateLFPose(robot.surface(leftFootSurface).X_0_s(robot));
  updateRHPose(robot.surface(rightHandSurface).X_0_s(robot));
  updateLHPose(robot.surface(leftHandSurface).X_0_s(robot));
}

void CoMQP::updateRFPose(sva::PTransformd const pose)
{
  if (!rfPoseUpdated){
      rfPose = pose;
      rfPoseUpdated = true;
  }
}

void CoMQP::updateLFPose(sva::PTransformd const pose)
{
  if (!lfPoseUpdated){
    lfPose = pose;
    lfPoseUpdated = true;
  }
}

void CoMQP::updateRHPose(sva::PTransformd const pose)
{
  if (!rhPoseUpdated){
    rhPose = pose;
    rhPoseUpdated = true;
  }
}

void CoMQP::updateLHPose(sva::PTransformd const pose)
{
  if (!lhPoseUpdated){
    lhPose = pose;
    lhPoseUpdated = true;
  }
}


void CoMQP::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement(
      {"CoMQP", "Gains"}, mc_rtc::gui::NumberInput("PG", [this]() { return P_PG; }, [this](double g) { P_PG = g; }),
      mc_rtc::gui::NumberInput("Force_rf", [this]() { return P_Force_rf; }, [this](double g) { P_Force_rf = g; }),
      mc_rtc::gui::NumberInput("Force_lf", [this]() { return P_Force_lf; }, [this](double g) { P_Force_lf = g; }),
      mc_rtc::gui::NumberInput("Force_rh", [this]() { return P_Force_rh; }, [this](double g) { P_Force_rh = g; }),
      mc_rtc::gui::NumberInput("Force_lh", [this]() { return P_Force_lh; }, [this](double g) { P_Force_lh = g; }),
      mc_rtc::gui::NumberInput("Wrench_rf", [this]() { return P_Wrench_rf; }, [this](double g) { P_Wrench_rf = g; }),
      mc_rtc::gui::NumberInput("Wrench_lf", [this]() { return P_Wrench_lf; }, [this](double g) { P_Wrench_lf = g; }),
      mc_rtc::gui::NumberInput("Wrench_rh", [this]() { return P_Wrench_rh; }, [this](double g) { P_Wrench_rh = g; }),
      mc_rtc::gui::NumberInput("Wrench_lh", [this]() { return P_Wrench_lh; }, [this](double g) { P_Wrench_lh = g; }),
      mc_rtc::gui::NumberInput("Desired rightHand normal force (N)", [this]() { return N_rh; }, [this](double g) { N_rh = g; }),
      mc_rtc::gui::NumberInput("Desired leftHand normal force (N)", [this]() { return N_lh; }, [this](double g) { N_lh = g; })
      );
}
void CoMQP::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeCategory({"CoMQP"});
}

void CoMQP::addToLogger(mc_rtc::Logger & logger)
{
  if(!inLogger_) // FIXME hack to ensure this is added only once
  {
    logger.addLogEntry("CoMQP_robot_mg", [this]() { return mg; });
    logger.addLogEntry("CoMQP_gains_PG", [this]() { return P_PG; });
    logger.addLogEntry("CoMQP_gains_Force_rf", [this]() { return P_Force_rf; });
    logger.addLogEntry("CoMQP_gains_Force_lf", [this]() { return P_Force_lf; });
    logger.addLogEntry("CoMQP_gains_Force_rh", [this]() { return P_Force_rh; });
    logger.addLogEntry("CoMQP_gains_Wrench_rf", [this]() { return P_Wrench_rf; });
    logger.addLogEntry("CoMQP_gains_Wrench_lf", [this]() { return P_Wrench_lf; });
    logger.addLogEntry("CoMQP_gains_Wrench_rh", [this]() { return P_Wrench_rh; });
    logger.addLogEntry("CoMQP_mu_y", [this]() { return mu_x_rh; });
    logger.addLogEntry("CoMQP_mu_z", [this]() { return mu_y_rh; });
    logger.addLogEntry("CoMQP_mu_sum", [this]() { return mu_x_rh+mu_y_rh; });

    logger.addLogEntry("CoMQP_pos", [this]() { return result().comPos; });
    logger.addLogEntry("CoMQP_rightFootForce", [this]() { return result().rightFootForce; });
    logger.addLogEntry("CoMQP_leftFootForce",  [this]() { return result().leftFootForce; });
    logger.addLogEntry("CoMQP_rightHandForce", [this]() { return result().rightHandForce; });
    logger.addLogEntry("CoMQP_desiredNormalForce_rh", [this]() { return N_rh; });
    logger.addLogEntry("CoMQP_desiredNormalForce_lh", [this]() { return N_lh; });
    logger.addLogEntry("RHVel", [this]() { return rhVel;});
    logger.addLogEntry("RHVel_local", [this]() { return localVel_rh;});
    logger.addLogEntry("RH_fs", [this]() { return rh_fs;});
    logger.addLogEntry("muN", [this]() { return muN;});
    logger.addLogEntry("RH_fs_rot", [this]() { return rh_fs_rot;});
    logger.addLogEntry("muXcalc", [this]() { return mu_x_calc;});

    logger.addLogEntry("CoMQP_RFPose", [this](){ return rfPose; });
    logger.addLogEntry("CoMQP_LFPose", [this](){ return lfPose; });
    logger.addLogEntry("CoMQP_RHPose", [this](){ return rhPose; });
    logger.addLogEntry("CoMQP_LHPose", [this](){ return lhPose; });
  }
  inLogger_ = true;
}

void CoMQP::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntry("CoMQP_robot_mg");
  logger.removeLogEntry("CoMQP_gains_PG");
  logger.removeLogEntry("CoMQP_gains_Force_rf");
  logger.removeLogEntry("CoMQP_gains_Wrench_rf");
  logger.removeLogEntry("CoMQP_gains_Force_lf");
  logger.removeLogEntry("CoMQP_gains_Wrench_lf");
  logger.removeLogEntry("CoMQP_gains_Force_rh");
  logger.removeLogEntry("CoMQP_gains_Wrench_rh");
  logger.removeLogEntry("CoMQP_gains_Force_lh");
  logger.removeLogEntry("CoMQP_gains_Wrench_lh");
  logger.removeLogEntry("CoMQP_mu_y");
  logger.removeLogEntry("CoMQP_mu_z");
  logger.removeLogEntry("CoMQP_mu_sum");

  logger.removeLogEntry("CoMQP_pos");
  logger.removeLogEntry("CoMQP_rightFootForce");
  logger.removeLogEntry("CoMQP_leftFootForce");
  logger.removeLogEntry("CoMQP_rightHandForce");
  logger.removeLogEntry("CoMQP_leftHandForce");
  logger.removeLogEntry("CoMQP_desiredNormalForce_rh");
  logger.removeLogEntry("CoMQP_desiredNormalForce_lh");
  logger.removeLogEntry("RHVel");
  logger.removeLogEntry("RHVel_local");
  logger.removeLogEntry("RH_fs");
  logger.removeLogEntry("muN");
  logger.removeLogEntry("RH_fs_rot");
  logger.removeLogEntry("muXcalc");

  logger.removeLogEntry("CoMQP_RFPose");
  logger.removeLogEntry("CoMQP_LFPose");
  logger.removeLogEntry("CoMQP_RHPose");
  logger.removeLogEntry("CoMQP_LHPose");
  
  inLogger_ = false;
}

Eigen::MatrixXd CoMQP::graspMatrixFromTranslation(Eigen::Vector3d trans) const
{
  const double Px = trans[0];
  const double Py = trans[1];
  const double Pz = trans[2];

  Eigen::MatrixXd graspMatrix;
  graspMatrix.resize(6,6);

  // clang-format off
  graspMatrix << 1., 0., 0., 0., 0., 0.,
                 0., 1., 0., 0., 0., 0.,
                 0., 0., 1., 0., 0., 0.,
                 0., -Pz, Py, 1., 0., 0.,
                 Pz, 0., -Px, 0., 1., 0.,
                 -Py, Px, 0., 0., 0., 1.;
  // clang-format on
  
  return graspMatrix;
}
