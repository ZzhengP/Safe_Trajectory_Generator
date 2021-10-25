#include <optFormulation/constraint.hpp>


namespace optimization {

bool MPCConstraint::init(const Eigen::VectorXd &q_min, const Eigen::VectorXd &q_max,
                         const Eigen::VectorXd &qd_min, const Eigen::VectorXd &qd_max,
                         double dsafe){

  dsafe_ = dsafe;
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
//  constraint.constraint_name_ = "end_joint_position";
//  constraint.lbA_.resize(dof_);
//  constraint.lbA_.setConstant(-1000);
//  constraint.ubA_.resize(dof_);
//  constraint.ubA_.setConstant(1000);
//  constraint.A_.resize(dof_,N_*dof_);
//  constraint.A_.setZero();
//  constraint.constraint_size_ = dof_ ;
//  constraint_container_.push_back(constraint);
//  constraint_nbr_+=constraint.constraint_size_;

  //   4. Table surface constraint
  constraint.constraint_name_ = "table_surface_constraint";
  constraint.lbA_.resize(2*(N_-1));
  constraint.ubA_.resize(2*(N_-1));
  constraint.A_.resize(2*(N_-1),N_*dof_);
  constraint.constraint_size_ = 2*(N_-1);
  constraint_container_.push_back(constraint);
  constraint_nbr_+=constraint.constraint_size_;

  std::cout << "constraint number : " << constraint_nbr_  << '\n';
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

bool MPCConstraint::update(const Eigen::VectorXd &state, Eigen::VectorXd q_des,const std::vector<Eigen::MatrixXd>& robotVerticesAugmented,
                           const std::vector<Eigen::MatrixXd>& plane,
                           const Eigen::MatrixXd & JacobianHorizon, const Eigen::VectorXd& qHorizonPrecedent)
{
  if (!mpc_params_.initialized){

      ROS_WARN_STREAM("Model Predictive parameters are null in task class !!!! ");
      return false;
  }

  if(!updateConstraintContainer(state, q_des,robotVerticesAugmented,
                                plane,
                                JacobianHorizon, qHorizonPrecedent)){
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




bool MPCConstraint::updateConstraintContainer(const Eigen::VectorXd &state, Eigen::VectorXd q_des,
                                              const std::vector<Eigen::MatrixXd>& robotVerticesAugmented,
                                              const std::vector<Eigen::MatrixXd>& plane,
                                              const Eigen::MatrixXd & JacobianHorizon, const Eigen::VectorXd& qHorizonPrecedent){

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



  //3 . end joint position constraint update
//  Eigen::VectorXd e;
//  e.resize(dof_), e.setConstant(5);
//  if (constraint_container_[2].constraint_name_ == "end_joint_position"){
//      constraint_container_[2].A_ = mpc_params_.Pu_.block((N_-1)*dof_,0,dof_,N_*dof_);
//      constraint_container_[2].lbA_ = q_des - mpc_params_.Px_.block((N_-1)*dof_,0,dof_,2*dof_) * state - e;
//      constraint_container_[2].ubA_ = q_des - mpc_params_.Px_.block((N_-1)*dof_,0,dof_,2*dof_) * state + e;
//  }else {
//      ROS_WARN_STREAM("Cannot end joint position constraint, order error in the vector list of task");
//      return false;
//  }

//   4. Table surface constraint update
  if (constraint_container_[2].constraint_name_ == "table_surface_constraint"){
       computeUpperBoundAndConstraint(state, robotVerticesAugmented,plane, JacobianHorizon, qHorizonPrecedent,2);

  }else{

    ROS_WARN_STREAM("Cannot update table surface constraint, order error in the vector list of task");
    return false;
  }

  return true;
}



bool MPCConstraint::computeUpperBoundAndConstraint(const Eigen::VectorXd &state,
                                                   const std::vector<Eigen::MatrixXd> &robotVerticesAugmented,
                                                   const std::vector<Eigen::MatrixXd> &plane,
                                                   const Eigen::MatrixXd &JacobianHorizon,
                                                   const Eigen::VectorXd &qHorizonPrecedent,
                                                   int index){

   double dsafe=0.1;

//   if (index == 2){
//     dsafe = 0.2;
//   }else {
//      dsafe = 0.2;
//    }

    // Resize the size of constraint according to the number of different m obstacle plane
    int m = plane.size();
    if (m!=1){
      ROS_WARN_STREAM("More than 1 obstacle");
      return false;
    }


    constraint_container_[index].lbA_.resize(2*m*(N_ - 1));
    constraint_container_[index].lbA_.setConstant(-100000);
    constraint_container_[index].ubA_.resize(2*m*(N_-1));
    constraint_container_[index].ubA_.setConstant(0);
    constraint_container_[index].A_.resize(2*m*(N_-1), dof_*N_);

    Eigen::VectorXd bLarge;
    bLarge.resize(2*(N_-1));
    bLarge.setZero();


    Eigen::MatrixXd nLarge;
    nLarge.resize(2*(N_-1), 6*(N_-1));
    nLarge.setZero();
    // ================= nJk*qk <= bk - dsafe - ak.rk + ak.Jk.qk ===================================
    // Because we pointing the plane to obstacle
    Eigen::VectorXd ar, aJq;
    ar.resize(2*(N_-1));
    aJq.resize(2*(N_-1));

    ar.setZero();
    aJq.setZero();

    // =====


    Eigen::MatrixXd n_jacobian;

    n_jacobian.resize(2*(N_-1), dof_*N_);
    n_jacobian.setZero();


    for (int nbr_plane(0); nbr_plane < m ; nbr_plane++){
      // this loop is actually not necessary, because m is always 1 since we treat each obstacle seperately

      Eigen::VectorXd plane_temp;
      plane_temp.resize(3);
      for (int k(0); k < N_-1; k++){


        plane_temp << plane[0](0,k), plane[0](1,k), plane[0](2,k);

        bLarge.segment(2*k,2) << plane[0](3,k) - dsafe, plane[0](3,k) - dsafe;



        ar.segment(2*k,2) << plane[0].block(0,k,3,1).transpose()*robotVerticesAugmented[0].block(0,k,3,1),
                             plane[0].block(0,k,3,1).transpose()*robotVerticesAugmented[0].block(0,k+1,3,1);



          n_jacobian.block(2*k, dof_*k, 2, 2*dof_) <<  plane_temp.transpose()*JacobianHorizon.block(0,0,3,dof_), 0,0,0 ,0,0,0,0,
                                                     0,0,0 ,0,0,0,0, plane_temp.transpose()*JacobianHorizon.block(0,0,3,dof_);
          aJq.segment(2*k,2) << plane_temp.transpose()*JacobianHorizon.block(0,0,3,7)*qHorizonPrecedent.segment(dof_*k,dof_),
                              plane_temp.transpose()*JacobianHorizon.block(0,0,3,7)*qHorizonPrecedent.segment(dof_*(k+1),dof_);

//        n_jacobian.block(2*k, dof_*k, 2, 2*dof_) <<  plane_temp.transpose()*JacobianHorizon.block(6*k,7*k,3,dof_), 0,0,0 ,0,0,0,0,
//                                                   0,0,0 ,0,0,0,0, plane_temp.transpose()*JacobianHorizon.block(6*(k+1),7*(k+1),3,dof_);
//        aJq.segment(2*k,2) << plane_temp.transpose()*JacobianHorizon.block(6*k,7*k,3,7)*qHorizonPrecedent.segment(dof_*k,dof_),
//                            plane_temp.transpose()*JacobianHorizon.block(6*(k+1),7*(k+1),3,7)*qHorizonPrecedent.segment(dof_*(k+1),dof_);

      }

      constraint_container_[index].A_ = n_jacobian*mpc_params_.Pu_;
      constraint_container_[index].ubA_ = bLarge - ar + aJq  - n_jacobian*mpc_params_.Px_*state;

    }

    return true;
}


}




