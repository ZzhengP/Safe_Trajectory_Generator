#include <optFormulation/task.hpp>

namespace optimization {


bool MPCTask::init(const Eigen::VectorXd & q_init){

  if (!mpc_params_.initialized){

      ROS_WARN_STREAM("Model Predictive parameters are null in task class !!!! ");
      return false;

  }

  H_.resize(N_*dof_,N_*dof_);
  H_.setIdentity();
  g_.resize(N_*dof_);
  g_.setZero();
  E_.resize(N_*dof_, N_*dof_);
  g_.resize(N_*dof_);
  // Add default tracking task: (joint position, joint velocity, joint acceleration)
  task task;
  task.task_name_="q_tracking";
  task.E_.resize(N_*dof_, N_*dof_);
  task.E_.setIdentity();
  task.f_.resize(N_*dof_);
  task.f_.setZero();
  task.task_des_.resize(dof_);
  task.task_des_ = q_init;
  task.weight = 10000;
  // Add joint position tracking task
  task_container_.push_back(task);

  // Add joint velocity tracking task
  task.task_name_="qd_tracking";
  task.weight = 1;
  task_container_.push_back(task);

  // Add joint acceleration tracking task
  task.weight = 10;
  task.task_name_="qdd_tracking";
  task_container_.push_back(task);



  return true;
}


bool MPCTask::update(const Eigen::VectorXd& S, const Eigen::VectorXd & q_des, const Eigen::VectorXd& qd_des, const Eigen::VectorXd& qdd_des){


  if (!mpc_params_.initialized){

      ROS_WARN_STREAM("Model Predictive parameters are null in task class !!!! ");
      return false;
  }

  if(!updateTaskContainer(S, q_des, qd_des, qdd_des)){

    ROS_ERROR_STREAM("Cannot update the vector list of task, pls verify that all task is well defined.");
    return false;
  }

  // Reset H and g to zeros
  H_.setZero();
  g_.setZero();

  for (auto it=task_container_.begin(); it!=task_container_.end(); it++){

    H_ += it->weight*it->E_.transpose()*it->E_;
    g_ += it->weight*it->E_.transpose()*it->f_;
  }


  return true;
}


bool MPCTask::updateTaskContainer(const Eigen::VectorXd& S, const Eigen::VectorXd & q_des, const Eigen::VectorXd& qd_des, const Eigen::VectorXd& qdd_des){

  // Update first the vector list of desired task
  //std::cout << " updating task" << '\n';

  // 1. joint position tracking update

  if(task_container_[0].task_name_=="q_tracking"){
     task_container_[0].task_des_ = q_des.head(dof_);
     task_container_[0].E_ = mpc_params_.Pu_;
     task_container_[0].f_ = mpc_params_.Px_*S - q_des;

   }else {
     ROS_WARN_STREAM("Cannot update joint position tracking task, order error in the vector list of task");
     return false;
  }

  // 2. joint velocity tracking update
  if(task_container_[1].task_name_=="qd_tracking"){
     task_container_[1].E_ = mpc_params_.Pudq_;
     task_container_[1].f_ = mpc_params_.Pxdq_*S ;
   }else {
     ROS_WARN_STREAM("Cannot update joint velocity tracking task, order error in the vector list of task");
     return false;
  }

  // 3. joint acceleration tracking update
  if(task_container_[2].task_name_=="qdd_tracking"){
     task_container_[2].task_des_ = qdd_des.head(dof_);
     task_container_[2].E_.setIdentity();
     task_container_[2].f_.setZero();
   }else {
     ROS_WARN_STREAM("Cannot update joint acceleration tracking task, order error in the vector list of task");
     return false;
  }

  //ROS_INFO_STREAM("Successfully update all task");

  return true;
}



}
