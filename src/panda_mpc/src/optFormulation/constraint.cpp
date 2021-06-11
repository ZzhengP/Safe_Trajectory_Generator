#include <optFormulation/constraint.hpp>


namespace optimization {

bool MPCConstraint::init(const Eigen::VectorXd &q_min, const Eigen::VectorXd &q_max,
                         const Eigen::VectorXd &qd_min, const Eigen::VectorXd &qd_max){

  if (!mpc_params_.initialized ){

      ROS_WARN_STREAM("Model Predictive parameters are null in task class !!!! ");
      return false;
  }

  constraint_nbr_= 0;
  // Add default constraint: Joint position and velocity limits
  constraint constraint;
  constraint.constraint_name_ = "joint_position_limit";
  constraint.lbA_.resize(dof_*N_);
  constraint.lbA_.setConstant(-10000);
  constraint.ubA_.resize(dof_*N_);
  constraint.ubA_.setConstant(10000);
  constraint.A_.resize(dof_*N_, dof_*N_);
  constraint.A_.setIdentity();
  constraint.constraint_size_ = dof_*N_;
  // 1. Joint position constraint
  constraint_container_.push_back(constraint);
  constraint_nbr_ += constraint.constraint_size_;

  // 2. Joint velocity constraint
  constraint.constraint_name_ = "joint_velocity_limit";
  constraint_container_.push_back(constraint);
  constraint_nbr_+= constraint.constraint_size_;

  // 3. End joint position constraint
  constraint.constraint_name_ = "end_joint_position";
  constraint.lbA_.resize(dof_);
  constraint.lbA_.setConstant(-1000);
  constraint.ubA_.resize(dof_);
  constraint.ubA_.setConstant(1000);
  constraint.A_.resize(dof_,N_*dof_);
  constraint.A_.setZero();
  constraint.constraint_size_ = dof_ ;
  constraint_container_.push_back(constraint);
  constraint_nbr_+=constraint.constraint_size_;

  // Resize the total number of constraints size
  lbA_.resize(constraint_nbr_);
  ubA_.resize(constraint_nbr_);
  A_.resize(constraint_nbr_, dof_*N_);

  lbA_.setConstant(-10000);
  ubA_.setConstant(10000);
  A_.setIdentity();

  q_min_mpc_.resize(N_*dof_), q_min_mpc_ = q_min;
  q_max_mpc_.resize(N_*dof_), q_max_mpc_ = q_max;
  qd_min_mpc_.resize(N_*dof_), qd_min_mpc_ = qd_min;
  qd_max_mpc_.resize(N_*dof_), qd_max_mpc_ = qd_max;

  return true;
}

bool MPCConstraint::update(const Eigen::VectorXd &state, Eigen::VectorXd q_des)
{
  if (!mpc_params_.initialized){

      ROS_WARN_STREAM("Model Predictive parameters are null in task class !!!! ");
      return false;
  }

  if(!updateConstraintContainer(state, q_des)){
    ROS_ERROR_STREAM("Cannot update the vector list of constraint, pls verify that all task is well defined.");
    return false;
  }

  int constraint_index = 0;
  for (auto it=constraint_container_.begin(); it!=constraint_container_.end(); ++it){

      lbA_.segment(constraint_index, it->constraint_size_) = it->lbA_;
      ubA_.segment(constraint_index, it->constraint_size_) = it->ubA_;
      A_.block(constraint_index,0,it->constraint_size_,dof_*N_) = it->A_;
      constraint_index += it->constraint_size_;
  }

  return true;

}



bool MPCConstraint::updateConstraintContainer(const Eigen::VectorXd &state, Eigen::VectorXd q_des){

    // 1. Joint position constraint update
  if (constraint_container_[0].constraint_name_ == "joint_position_limit"){
      constraint_container_[0].A_ = mpc_params_.Pu_;
      constraint_container_[0].lbA_ = q_min_mpc_ - mpc_params_.Px_ * state;
      constraint_container_[0].ubA_ = q_max_mpc_ - mpc_params_.Px_ * state;
  }else {
    ROS_WARN_STREAM("Cannot update joint position constraint, order error in the vector list of task");
    return false;
}

  // 2. Joint velocity constraint update
  if (constraint_container_[1].constraint_name_ == "joint_velocity_limit"){
      constraint_container_[1].A_ = mpc_params_.Pudq_;
      constraint_container_[1].lbA_ = qd_min_mpc_ - mpc_params_.Pxdq_ * state;
      constraint_container_[1].ubA_ = qd_max_mpc_ - mpc_params_.Pxdq_ * state;
  }else {
      ROS_WARN_STREAM("Cannot update joint velocity constraint, order error in the vector list of task");
      return false;
  }

  // 3. end joint position constraint update
//  Eigen::MatrixXd Pu, Px;
//  Pu.resize(N_*dof_, N_*dof_);
//  Pu = mpc_params_.Pu_;

//  Px.resize(N_*dof_, 2*dof_);
//  Px = mpc_params_.Px_;
  Eigen::VectorXd e;
  e.resize(dof_), e.setConstant(1);
  if (constraint_container_[2].constraint_name_ == "end_joint_position"){
      constraint_container_[2].A_ = mpc_params_.Pu_.block((N_-1)*dof_,0,dof_,N_*dof_);
      constraint_container_[2].lbA_ = q_des - mpc_params_.Px_.block((N_-1)*dof_,0,dof_,2*dof_) * state - e;
      constraint_container_[2].ubA_ = q_des - mpc_params_.Px_.block((N_-1)*dof_,0,dof_,2*dof_) * state + e;
  }else {
      ROS_WARN_STREAM("Cannot end joint position constraint, order error in the vector list of task");
      return false;
  }

  return true;
}

}




