#include <robot/robot_mpc_model.h>

namespace robot {


bool RobotMPcModel::InitMPCParameter(ros::NodeHandle &node_handle){

  // INITIALIZE LINEAR MODEL PARAMETERS

  setPx();
  setPu();
  setPxDq();
  setPuDq();

  if(mpc_params == nullptr){
    ROS_WARN_STREAM("model predictive parameter structure is null !!!");
    return false;
  }

  for (unsigned int i(0); i<N_; i++){
    mpc_params->q_horizon_.segment(i*dof, dof) = q_init_;
    mpc_params->qd_horizon_.segment(i*dof,dof) = qd_init_;
    mpc_params->x_horizon_.segment(i*3,3) = x_curr_;
    mpc_params->J_horizon_.block(6*i,dof*i,6,dof) = J_.data;
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

    mpc_params->J_horizon_.block(i*6, i*dof, 6, dof) = J_temp.data;

  }
}




}
