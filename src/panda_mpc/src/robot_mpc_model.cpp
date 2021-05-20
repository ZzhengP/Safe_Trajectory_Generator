#include <robot/robot_mpc_model.h>

namespace robot {


bool RobotMPcModel::InitMPCParameter(ros::NodeHandle &node_handle){

  // INITIALIZE LINEAR MODEL PARAMETERS
  Px_.resize(N_*dof, 2*dof);
  Px_.setZero();
  Pu_.resize(N_*dof, N_*dof);
  Pu_.setZero();

  Pxdq_.resize(N_*dof, 2*dof);
  Pxdq_.setZero();
  Pudq_.resize(N_*dof, N_*dof);
  Pudq_.setZero();

  q_horizon_.resize(N_*dof);
  qd_horizon_.resize(N_*dof);
  x_horizon_.resize(3*N_);
  J_horizon_.resize(N_*6, N_*dof);
  J_horizon_.setZero();
  for (unsigned int i(0); i<N_; i++){
    q_horizon_.segment(i*dof, dof) = q_init_;
    qd_horizon_.segment(i*dof,dof) = qd_init_;
    x_horizon_.segment(i*3,3) = x_curr_;
    J_horizon_.block(6*i,dof*i,6,dof) = J_.data;
  }

  return true;
}


void RobotMPcModel::computeJacobianHorizon(const Eigen::VectorXd &q_horizon){

  KDL::JntArray q_temp;
  KDL::Jacobian J_temp;

  q_temp.resize(dof);
  q_temp.data.setZero();
  J_temp.resize(dof);
  J_temp.data.setZero();

  for (size_t i(0); i<N_; i++){


    q_temp.data = q_horizon.segment(i*dof, dof);

    JntToJac(q_temp, J_temp);

    J_horizon_.block(i*6, i*dof, 6, dof) = J_temp.data;

  }
}


}
