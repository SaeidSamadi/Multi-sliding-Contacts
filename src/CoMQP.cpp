#include "CoMQP.h"

#include <mc_rtc/logging.h>
using namespace std;

CoMQP::CoMQP(const mc_rbdyn::Robot & robot, const mc_rtc::Configuration & config)
{
  numVar_ = 3 + 6 * 3 + 1;

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

  sliding1.resize(6, 6);
  sliding1.setZero();
  sliding1(0, 0) = 1.0;
  sliding1(1, 1) = 1.0;
  sliding1(2, 2) = 1.0;
  sliding2.resize(6, 6);
  sliding2.setZero();

  sliding1_check.resize(6, 6);
  sliding1_check.setZero();
  sliding1_check(0, 2) = 1.0;
  sliding1_check(1, 1) = 1.0;
  sliding1_check(2, 0) = -1.0;
  sliding2_check.resize(6, 6);
  sliding2_check.setZero();

  sliding.resize(6, 6);
  sliding.setZero();

  sliding2_tmp.resize(6, 6);
  sliding2_tmp.setZero();

  sliding_tmp.resize(6, 6);
  sliding_tmp.setZero();

  A.resize(12, 21);
  A.setZero();
  
  A_st.resize(12, 22);
  A_st.setZero();

  UBmat_rf.resize(6, 6);
  UBmat_rf.setZero();
  UBmat_lf.resize(6, 6);
  UBmat_lf.setZero();
  UBmat_rh.resize(6, 6);
  UBmat_rh.setZero();
  UBmat_rh_tmp.resize(6, 6);
  UBmat_rh_tmp.setZero();

  LBmat_rf.resize(6, 6);
  LBmat_rf.setZero();
  LBmat_lf.resize(6, 6);
  LBmat_lf.setZero();
  LBmat_rh.resize(6, 6);
  LBmat_rh.setZero();
  LBmat_rh_tmp.resize(6, 6);
  LBmat_rh_tmp.setZero();

  b.resize(12);
  b.setZero();

  //b_st.resize(13);
  //b_st.setZero();

  Ineq_mat1.resize(6, 21);
  Ineq_mat2.resize(6, 21);
  Ineq_mat3.resize(6, 21);
  Ineq_mat4.resize(6, 21);
  Ineq_mat5.resize(6, 21);
  Ineq_mat6.resize(6, 21);
  Ineq_mat1.setZero();
  Ineq_mat2.setZero();
  Ineq_mat3.setZero();
  Ineq_mat4.setZero();
  Ineq_mat5.setZero();
  Ineq_mat6.setZero();

  Ineq_max_rf.resize(4, 21);
  Ineq_min_rf.resize(4, 21);
  Ineq_max_lf.resize(4, 21);
  Ineq_min_lf.resize(4, 21);
  Ineq_max_rh.resize(4, 21);
  Ineq_min_rh.resize(4, 21);
  Ineq_max_nonRotate.resize(4, 6);
  Ineq_max_rf.setZero();
  Ineq_min_rf.setZero();
  Ineq_max_lf.setZero();
  Ineq_min_lf.setZero();
  Ineq_max_rh.setZero();
  Ineq_min_rh.setZero();
  Ineq_max_nonRotate.setZero();

  G.resize(60, 21);
  G.setZero();
  
  G_st.resize(61, 22);
  G_st.setZero();

  h.resize(60);
  h.setZero();
  
  h_st.resize(61);
  h_st.setZero();

  P.resize(22, 22);
  P.setZero();


  Y_desired.resize(22);
  Y_desired.setZero();

  q.resize(22);
  q.setZero();


  solver_.problem(nrVar(), nrEq(), nrInEq());
}

bool CoMQP::solve(const mc_rbdyn::Robot & robot)
{
  const double Px_rf = robot.surface(rightFootSurface).X_0_s(robot).translation().x();
  const double Py_rf = robot.surface(rightFootSurface).X_0_s(robot).translation().y();
  const double Pz_rf = robot.surface(rightFootSurface).X_0_s(robot).translation().z();
  const double Px_lf = robot.surface(leftFootSurface).X_0_s(robot).translation().x();
  const double Py_lf = robot.surface(leftFootSurface).X_0_s(robot).translation().y();
  const double Pz_lf = robot.surface(leftFootSurface).X_0_s(robot).translation().z();
  const double Px_rh = robot.surface(rightHandSurface).X_0_s(robot).translation().x();
  const double Py_rh = robot.surface(rightHandSurface).X_0_s(robot).translation().y();
  const double Pz_rh = robot.surface(rightHandSurface).X_0_s(robot).translation().z();
  const double Px_lh = robot.surface(leftHandSurface).X_0_s(robot).translation().x();
  const double Py_lh = robot.surface(leftHandSurface).X_0_s(robot).translation().y();
  const double Pz_lh = robot.surface(leftHandSurface).X_0_s(robot).translation().z();
  const auto R_rf = robot.surface(rightFootSurface).X_0_s(robot).rotation();
  const auto R_lf = robot.surface(leftFootSurface).X_0_s(robot).rotation();
  const auto R_rh = robot.surface(rightHandSurface).X_0_s(robot).rotation();
  const auto R_lh = robot.surface(leftHandSurface).X_0_s(robot).rotation();
  Eigen::Matrix3d constRot_rh; // constRot is to rotate the body local axis to desired local
  constRot_rh << 0.0,  0.0, -1.0,
                 1.0,  0.0,  0.0,
                 0.0, -1.0,  0.0;
  const auto bodyName_rh = robot.surface(rightHandSurface).bodyName();
  //mc_rtc::log::info("BodyName: {}", bodyName);
  const auto body_PT = robot.bodyPosW(bodyName_rh);
  const auto bodyRot = constRot_rh * body_PT.rotation();

  //mc_rtc::log::info("surfaceRot: \n {}", R_rh);
  //mc_rtc::log::info("Body Rotation Matrix: \n {}", bodyRot);
  Eigen::Matrix6d Rot_rf;
  Rot_rf.setZero();
  Rot_rf.block<3, 3>(0, 0) = R_rf;
  Rot_rf.block<3, 3>(3, 3) = R_rf;
  Eigen::Matrix6d Rot_lf;
  Rot_lf.setZero();
  Rot_lf.block<3, 3>(0, 0) = R_lf;
  Rot_lf.block<3, 3>(3, 3) = R_lf;
  Eigen::Matrix6d Rot_rh;
  Rot_rh.setZero();
  Rot_rh.block<3, 3>(0, 0) = bodyRot; //R_rh; //bodyRot;
  Rot_rh.block<3, 3>(3, 3) = bodyRot; //R_rh; //bodyRot;
  Eigen::Matrix6d Rot_lh;
  Rot_lh.setZero();
  Rot_lh.block<3, 3>(0, 0) = R_lh;
  Rot_lh.block<3, 3>(3, 3) = R_lh;
  //const auto Rot_rh_dual = robot.surface(rightHandSurface).X_0_s(robot).dualMatrix();
  //LOG_INFO("RotMat:" << Rot_rh.transpose() << endl); 
  //LOG_INFO("DualMat:" << Rot_rh_dual << endl); 
  // XXX inefficient
  // clang-format off
  E_rf << 1., 0., 0., 0., 0., 0.,
         0., 1., 0., 0., 0., 0.,
         0., 0., 1., 0., 0., 0.,
         0., -Pz_rf, Py_rf, 1., 0., 0.,
         Pz_rf, 0., -Px_rf, 0., 1., 0.,
         -Py_rf, Px_rf, 0., 0., 0., 1.;

  E_lf << 1., 0., 0., 0., 0., 0.,
         0., 1., 0., 0., 0., 0.,
         0., 0., 1., 0., 0., 0.,
         0., -Pz_lf, Py_lf, 1., 0., 0.,
         Pz_lf, 0., -Px_lf, 0., 1., 0.,
         -Py_lf, Px_lf, 0., 0., 0., 1.;

  E_rh <<
          1., 0., 0., 0., 0., 0.,
          0., 1., 0., 0., 0., 0.,
          0., 0., 1., 0., 0., 0.,
          0., -Pz_rh, Py_rh, 1., 0., 0.,
          Pz_rh, 0., -Px_rh, 0., 1., 0.,
          -Py_rh, Px_rh, 0., 0., 0., 1.;
  E_lh <<
          1., 0., 0., 0., 0., 0.,
          0., 1., 0., 0., 0., 0.,
          0., 0., 1., 0., 0., 0.,
          0., -Pz_lh, Py_lh, 1., 0., 0.,
          Pz_lh, 0., -Px_lh, 0., 1., 0.,
          -Py_lh, Px_lh, 0., 0., 0., 1.;
  // clang-format on

  // XXX measure mu_y and mu_z from force sensor here.
  // Usually Saeid sets it according to the trajectory
  // Let's see what's better
  // {{{
  // const auto & rh_fs = robot.forceSensor(rightHandForceSensor).worldWrenchWithoutGravity(robot);
  // Eigen::Vector3d mu = rh_fs.force()/N;
  // mu_y = mu.y();
  // mu_z = mu.z();
  // }}}
  sva::MotionVecd rhVel = robot.bodyVelW("r_wrist");
  //sva::MotionVecd rhVel_b = robot.bodyVelB("r_wrist");
 // sva::MotionVecd rhVel_b = robot.bodyVelB("r_wrist");
  //sva::MotionVecd rhVel = robot.bodyVelW("___"); XXX Other Sliding contacts
  //Eigen::Vector3d CoMPos = robot.com();
  //double velNorm = rhVel.linear().norm();
 // Eigen::Vector3d bb;
 // bb << rhVel_b.linear().x(),rhVel_b.linear().y(), 0.;
 // Eigen::Vector3d aa;
 // aa = R_rh * bb; 
  //double velNorm = aa.norm();
  Eigen::Vector3d RWristVel, RWristVelB, RotatedRWB, localVel;
  RWristVel << rhVel.linear().x(), rhVel.linear().y(), rhVel.linear().z(); //Global = [0, 0, vel]
  localVel = R_rh * RWristVel; // (x)<- || (y)|_ 
  double velNorm = Eigen::Vector3d{-localVel(1),-localVel(0), 0.}.norm();
  //RWristVelB << rhVel_b.linear().x(),rhVel_b.linear().y(), rhVel_b.linear().z(); 

 // LOG_INFO("RW_Vel"<<RWristVel<<endl);
 // LOG_INFO("RW_VelB"<<RWristVelB<<endl);
 // LOG_INFO("localVel"<<localVel<<endl);

  if(velNorm < 10e-5)
  {
    mu_x = mu_rh/2.;
    mu_y = mu_rh/2.;
  }
  else
  {
    mu_x = mu_rh * fabs(localVel(0)) / velNorm;
    mu_y = mu_rh * fabs(localVel(1)) / velNorm;
  }

  sliding2_check(0, 0) = mu_x;
  sliding2_check(1, 0) = mu_y;
  sliding2(0, 2) = mu_x;
  sliding2(1, 2) = mu_y;
  sliding_tmp = sliding1 - sliding2; 
  sliding_tmp = sliding_tmp * Rot_rh;
  if(CoMQP::desiredNormalForce() < 2){
  sliding = sliding1_check - sliding2_check;
  }
  else{
  sliding = sliding_tmp;
  }
  /* XXX For Horizental slidings
  double velNorm_H = Eigen::Vector3d{rhVel_H.linear().x(), rhVel_H.linear().y(), 0.}.norm();
  if(velNorm_H < 10e-5)
  {
    mu_x_H = mu_CONTACT/2.;
    mu_y_H = mu_rh/2.;
  }
  else
  {
    mu_x_H = mu_rh * fabs(rhVel.linear().y()) / velNorm;
    mu_y_H = mu_rh * fabs(rhVel.linear().z()) / velNorm;
  }

  sliding2_H(0, 2) = mu_x_H;
  sliding2_H(1, 2) = mu_y_H;
  sliding_H = sliding1 - sliding2_H;
  */

  A_st.block<6, 3>(0, 0) = E_m1;
  A_st.block<6, 6>(0, 3) = E_rf;
  A_st.block<6, 6>(0, 9) = E_lf;
  A_st.block<6, 6>(0, 15) = E_rh;
  //A_st.block<6, 6>(0, 21) = E_rh; XXX A_st: 12x21 -> 12x27 (adding a contact)
  A_st.block<6, 6>(6, 15) = sliding;
  //A_st.block<6, 6>(12, relatedContact) = sliding; XXX b: 12x1 -> 18x1
  // (12,15):rh / lf:9 / rf:3 / lh:21               XXX A_st: 12x21 -> 18x21 (adding Sliding Contact)

  Eigen::Matrix6d IneqVarMat;
  IneqVarMat.setZero();
  IneqVarMat(0, 2) = mu_rf;
  IneqVarMat(1, 2) = mu_rf;
  IneqVarMat(3, 2) = Y_rf;
  IneqVarMat(4, 2) = X_rf;

  UBmat_ineq = Eigen::Matrix6d::Identity() - IneqVarMat;
  UBmat_ineq(5, 5) = 0.0;
  LBmat_ineq = -Eigen::Matrix6d::Identity() - IneqVarMat;
  LBmat_ineq(5, 5) = 0.0;
  
  //UBmat_rf = UB_ineq * Rot_rf;
  //UBmat_lf = UB_ineq * Rot_lf;

  // Sliding Contact: tau_y <= X(-f_x) && tau_z <= y(-f_x)
  UBmat_rh = UBmat_ineq;
  UBmat_rh.block<3, 3>(0, 0) = Eigen::Matrix3d::Zero(); //needed for sliding contacts
  UBmat_rh.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero(); //needed for sliding contacts
  UBmat_rh = UBmat_rh * Rot_rh;

  LBmat_rh = LBmat_ineq;
  LBmat_rh.block<3, 3>(0, 0) = Eigen::Matrix3d::Zero(); //needed for sliding contacts
  LBmat_rh.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero(); //needed for sliding contacts
  LBmat_rh = LBmat_rh * Rot_rh;

  UBmat_rh_tmp.setZero();
  //mc_rtc::log::info("zeroMat: \n {}", UBmat_rh_tmp);
  UBmat_rh_tmp(3, 2) = -Y_rh;
  UBmat_rh_tmp(4, 2) = -X_rh;
  UBmat_rh_tmp(3, 3) = 1.0;
  UBmat_rh_tmp(4, 4) = 1.0;
  //mc_rtc::log::info("local_BeforeRotation: \n {}", UBmat_rh_tmp);
  //mc_rtc::log::info("RotationMat: \n {}", Rot_rh);
  UBmat_rh_tmp = UBmat_rh_tmp * Rot_rh;
  //mc_rtc::log::info("Rotated_UB: \n {}", UBmat_rh_tmp);
  /*
   If Sliding horizental:
  UBmat_lf(3, 2) = Y_lf;
  UBmat_lf(4, 2) = X_lf;
  LBmat_lf = UBmat_lf;
   *  By adding new contact: add IneqMat 7 and 8
   *  These are for [f tau_x tau_y]
   */
  b.head<6>() = E_m2.head<6>();
  b(8) = N;
  //b_st.head<12>() = b;

  Ineq_mat1.block<6, 6>(0, 3) = UBmat_ineq;
  Ineq_mat2.block<6, 6>(0, 3) = LBmat_ineq;
  Ineq_mat3.block<6, 6>(0, 9) = UBmat_ineq;
  Ineq_mat4.block<6, 6>(0, 9) = LBmat_ineq;
  if(CoMQP::desiredNormalForce() > 2){
  Ineq_mat5.block<6, 6>(0, 15) = UBmat_rh; // So for Sliding contacts: Zero() matrix
  Ineq_mat6.block<6, 6>(0, 15) = LBmat_rh; 
  }
  else{
  }

  double coef_rf = -mu_rf * (X_rf + Y_rf);
  double coef_lf = -mu_lf * (X_lf + Y_lf);
  double coef_rh = -mu_rh * (X_rh + Y_rh);
  Eigen::MatrixXd tauZ_max_rf(4, 6), tauZ_min_rf(4, 6);
  Eigen::MatrixXd tauZ_max_lf(4, 6), tauZ_min_lf(4, 6);
  Eigen::MatrixXd tauZ_max_rh(4, 6), tauZ_min_rh(4, 6);
  Eigen::MatrixXd tauZ_max_lh(4, 6), tauZ_min_lh(4, 6);
  tauZ_max_rf <<
           Y_rf, X_rf, coef_rf, mu_rf, mu_rf, 1.,
           -Y_rf, X_rf, coef_rf, -mu_rf, mu_rf, 1.,
           Y_rf, -X_rf, coef_rf, mu_rf, -mu_rf, 1.,
           -Y_rf, -X_rf, coef_rf, -mu_rf, -mu_rf, 1.;
  tauZ_min_rf <<
           Y_rf, X_rf, coef_rf, -mu_rf, -mu_rf, -1.,
           -Y_rf, X_rf, coef_rf, mu_rf, -mu_rf, -1.,
           Y_rf, -X_rf, coef_rf, -mu_rf, mu_rf, -1.,
           -Y_rf, -X_rf, coef_rf, mu_rf, mu_rf, -1.;
  tauZ_max_lf <<
           Y_lf, X_lf, coef_lf, mu_lf, mu_lf, 1.,
           -Y_lf, X_lf, coef_lf, -mu_lf, mu_lf, 1.,
           Y_lf, -X_lf, coef_lf, mu_lf, -mu_lf, 1.,
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
  // clang-format off
  Ineq_max_rf.block<4,6>(0,3) = tauZ_max_rf;
  Ineq_min_rf.block<4,6>(0,3) = tauZ_min_rf;
  Ineq_max_lf.block<4,6>(0,9) = tauZ_max_lf;
  Ineq_min_lf.block<4,6>(0,9) = tauZ_min_lf;
  
  Eigen::MatrixXd desiredMat(4, 6);
  desiredMat << // Vertical contact
            -coef_rh, X_rh, Y_rh, 1, -mu_rh, -mu_rh,
            -coef_rh, X_rh, -Y_rh, 1, -mu_rh, mu_rh,
            -coef_rh, -X_rh, Y_rh, 1, mu_rh, -mu_rh,
            -coef_rh, -X_rh, -Y_rh, 1, mu_rh, mu_rh;
 // if(CoMQP::desiredNormalForce() < 2){
 // Ineq_max_rh.block<4,6>(0,15) << // Vertical contact
 //           -coef_rh, X_rh, Y_rh, 1, -mu_rh, -mu_rh,
 //           -coef_rh, X_rh, -Y_rh, 1, -mu_rh, mu_rh,
 //           -coef_rh, -X_rh, Y_rh, 1, mu_rh, -mu_rh,
 //           -coef_rh, -X_rh, -Y_rh, 1, mu_rh, mu_rh;
 // }
 // else{
  Ineq_max_rh.block<4,6>(0,15) = tauZ_min_rh * Rot_rh; //XXX min is used for max!!!
 // mc_rtc::log::info("tauZ_nonRotate: \n {}", tauZ_max_rh);
 // mc_rtc::log::info("RotationMat: \n {}", Rot_rh);
 // mc_rtc::log::info("tauZ_desiredForm: \n {}", desiredMat);
 // mc_rtc::log::info("tauZ_rotated: \n {}", Ineq_max_rh.block<4,6>(0,15));
 // }

  if(CoMQP::desiredNormalForce() < 2){
  Ineq_min_rh.block<4,6>(0,15) <<
           coef_rh, -X_rh, -Y_rh, 1, -mu_rh, -mu_rh,
           coef_rh, -X_rh, Y_rh, 1, -mu_rh, mu_rh,
           coef_rh, X_rh, -Y_rh, 1, mu_rh, -mu_rh,
           coef_rh, X_rh, Y_rh, 1, mu_rh, mu_rh;
  }
  else{
  Ineq_min_rh.block<4,6>(0,15) = tauZ_max_rh * Rot_rh;
  }
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
  G_st.block<60, 21>(0, 0) = G;
  
  for (int i=0; i<20*CoMQP::k; i++){
  double zeta = G.block<1,21>(i,0).norm();
  G_st(i,3+6*k) = zeta;
  //cout<<G_st(i,3+6*k)<<endl;
  };
  G_st(20*k, 3+6*k) = -1.;

  h_st(2) = fz_max; //For Fixed contacts set it for normal component; Horizental: f_z, Vertical Fix: f_x
  h_st(8) = -fz_min;
  h_st(14) = fz_max;
  h_st(20) = -fz_min;
  //h_st.head<60>() = h;
  
  P.block<3, 3>(0, 0) = P_PG * Eigen::Matrix3d::Identity();
  P.block<3, 3>(3, 3) = P_Force_rf * Eigen::Matrix3d::Identity();
  P.block<3, 3>(6, 6) = P_Wrench_rf * Eigen::Matrix3d::Identity();
  P.block<3, 3>(9, 9) = P_Force_lf * Eigen::Matrix3d::Identity();
  P.block<3, 3>(12, 12) = P_Wrench_lf * Eigen::Matrix3d::Identity();
  P.block<3, 3>(15, 15) = P_Force_rh * Eigen::Matrix3d::Identity();
  P.block<3, 3>(18, 18) = P_Wrench_rh * Eigen::Matrix3d::Identity();
  //P.block<3, 3>(15, 15) = P_Force_rh * Eigen::Matrix3d::Identity();  // size P: 22x22 -> 28x28
  //P.block<3, 3>(18, 18) = P_Wrench_rh * Eigen::Matrix3d::Identity(); // Also set nVar_
  P(21,21) = P_Radius; //Put it at last
  P.Identity(22,22);
  P = 2 * P;

  //const Eigen::Vector3d & centroid = supportPolygon.centralPoint();
  //double com_x_d = centroid.x();
  //double com_y_d = centroid.y();
  
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
  Eigen::Vector3d f_rh_d{N, mu_x * N, mu_y * N}; //XXX Set sliding Condition here
  Eigen::Vector3d tau_rh_d = Eigen::Vector3d{Px_rh, Py_rh, Pz_rh}.cross(f_rh_d);
  
  //Y_desired.block<3, 1>(0, 0) = PG_d;
  //Y_desired.block<3, 1>(3, 0) = f_rf_d;
  //Y_desired.block<3, 1>(6, 0) = tau_rf_d;
  //Y_desired.block<3, 1>(9, 0) = f_lf_d;
  //Y_desired.block<3, 1>(12, 0) = tau_lf_d;
  Y_desired.block<3, 1>(15, 0) = f_rh_d;     // uncomment or add for sliding contact
  Y_desired.block<3, 1>(18, 0) = tau_rh_d;   // q_size
  q = -2* Y_desired;
  q(21) = 1.;
  

  /*
  cout<<"A_st size" << A_st.rows()<< "x" << A_st.cols() << endl;
  cout<<"b size" << b.rows()<< "x" << b.cols() << endl;
  cout<<"G_st size" << G_st.rows()<< "x" << G_st.cols() << endl;
  cout<<"h_st size" << h_st.rows()<< "x" << h_st.cols() << endl;
  */


  if(debug_)
  {
    LOG_INFO("mu_x: " << mu_x);
    LOG_INFO("mu_y: " << mu_y);
    LOG_INFO("P:");
    LOG_INFO(P);
    LOG_INFO("\n\nq:");
    LOG_INFO(q.transpose());
    LOG_INFO("\n\nA:");
    LOG_INFO(A);
    LOG_INFO("\n\nb:");
    LOG_INFO(b.transpose());
    LOG_INFO("\n\nG:");
    LOG_INFO(G);
    LOG_INFO("\n\nh:");
    LOG_INFO(h.transpose());
  }

  bool ret = solver_.solve(P, q, A_st, b, G_st, h_st);
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
  //LOG_SUCCESS("info : " << r.transpose() << " foobar")
  //cout<<r(21)<<endl;
  //cout<<r.size()<<endl;
  return res;
}

int CoMQP::errorCode() const
{
  solver_.fail();
}

void CoMQP::errorMessage() const
{
  std::ostringstream stream;
  //solver_.inform(stream);
  LOG_ERROR(stream.str());
}

void CoMQP::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement(
      {"CoMQP", "Gains"}, mc_rtc::gui::NumberInput("PG", [this]() { return P_PG; }, [this](double g) { P_PG = g; }),
      mc_rtc::gui::NumberInput("Force_rf", [this]() { return P_Force_rf; }, [this](double g) { P_Force_rf = g; }),
      mc_rtc::gui::NumberInput("Force_lf", [this]() { return P_Force_lf; }, [this](double g) { P_Force_lf = g; }),
      mc_rtc::gui::NumberInput("Force_rh", [this]() { return P_Force_rh; }, [this](double g) { P_Force_rh = g; }),
      mc_rtc::gui::NumberInput("Wrench_rf", [this]() { return P_Wrench_rf; }, [this](double g) { P_Wrench_rf = g; }),
      mc_rtc::gui::NumberInput("Wrench_lf", [this]() { return P_Wrench_lf; }, [this](double g) { P_Wrench_lf = g; }),
      mc_rtc::gui::NumberInput("Wrench_rh", [this]() { return P_Wrench_rh; }, [this](double g) { P_Wrench_rh = g; }),
      mc_rtc::gui::NumberInput("Desired normal force (N)", [this]() { return N; }, [this](double g) { N = g; })
      );
}
void CoMQP::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeCategory({"CoMQP"});
}

void CoMQP::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry("CoMQP_robot_mg", [this]() { return mg; });
  logger.addLogEntry("CoMQP_gains_PG", [this]() { return P_PG; });
  logger.addLogEntry("CoMQP_gains_Force_rf", [this]() { return P_Force_rf; });
  logger.addLogEntry("CoMQP_gains_Force_lf", [this]() { return P_Force_lf; });
  logger.addLogEntry("CoMQP_gains_Force_rh", [this]() { return P_Force_rh; });
  logger.addLogEntry("CoMQP_gains_Wrench_rf", [this]() { return P_Wrench_rf; });
  logger.addLogEntry("CoMQP_gains_Wrench_lf", [this]() { return P_Wrench_lf; });
  logger.addLogEntry("CoMQP_gains_Wrench_rh", [this]() { return P_Wrench_rh; });
  logger.addLogEntry("CoMQP_mu_x", [this]() { return mu_x; });
  logger.addLogEntry("CoMQP_mu_y", [this]() { return mu_y; });
  logger.addLogEntry("CoMQP_mu_sum", [this]() { return mu_x+mu_y; });

  logger.addLogEntry("CoMQP_pos", [this]() { return result().comPos; });
  logger.addLogEntry("CoMQP_rightFootForce", [this]() { return result().rightFootForce; });
  logger.addLogEntry("CoMQP_leftFootForce",  [this]() { return result().leftFootForce; });
  logger.addLogEntry("CoMQP_rightHandForce", [this]() { return result().rightHandForce; });
  logger.addLogEntry("CoMQP_desiredNormalForce", [this]() { return N; });
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
  logger.removeLogEntry("CoMQP_mu_x");
  logger.removeLogEntry("CoMQP_mu_y");
  logger.removeLogEntry("CoMQP_mu_sum");

  logger.removeLogEntry("CoMQP_pos");
  logger.removeLogEntry("CoMQP_rightFootForce");
  logger.removeLogEntry("CoMQP_leftFootForce");
  logger.removeLogEntry("CoMQP_rightHandForce");
  logger.removeLogEntry("CoMQP_desiredNormalForce");
}
