#include <robot/robot_mpc_model.h>

namespace robot {


bool RobotMPcModel::InitMPCParameter(ros::NodeHandle &node_handle){

  // INITIALIZE LINEAR MODEL PARAMETERS

  setPx();
  setPu();
  setPxDq();
  setPuDq();

  if(!mpc_params.initialized){
    ROS_WARN_STREAM("model predictive parameter structure is null !!!");
    return false;
  }

  for (unsigned int i(0); i<N_; i++){
    mpc_params.q_horizon_.segment(i*dof, dof) = q_init_;
    mpc_params.qd_horizon_.segment(i*dof,dof) = qd_init_;
    mpc_params.x_horizon_.segment(i*3,3) = x_curr_;
    mpc_params.J_horizon_.block(6*i,dof*i,6,dof) = J_.data;
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

    mpc_params.J_horizon_.block(i*6, i*dof, 6, dof) = J_temp.data;

  }
}

Eigen::MatrixXd RobotMPcModel::computeTipPositionHorizon(const Eigen::VectorXd &q_horizon,
                                                         int robot_vertices){

    Eigen::MatrixXd robotVerticesAugmented;
    robotVerticesAugmented.resize(3, N_*robot_vertices);

    KDL::JntArray q_in;
    KDL::Frame p_out;

    q_in.resize(dof);

    for (int i(0); i<N_ ;i++){
      q_in.data = q_horizon.segment(dof*i,dof);
      JntToCart(q_in, p_out);
      robotVerticesAugmented.block(0,i,3,1) << p_out.p[0], p_out.p[1], p_out.p[2];
      mpc_params.x_horizon_.segment(3*i,3) << p_out.p[0], p_out.p[1], p_out.p[2];

    }


    return robotVerticesAugmented;
}

void RobotMPcModel::update(Eigen::VectorXd state, Eigen::VectorXd solution){

  mpc_params.q_horizon_ = mpc_params.Px_*state + mpc_params.Pu_*solution;
  mpc_params.qd_horizon_ = mpc_params.Pxdq_*state + mpc_params.Pudq_*solution;
  computeJacobianHorizon(mpc_params.q_horizon_);


}


}
