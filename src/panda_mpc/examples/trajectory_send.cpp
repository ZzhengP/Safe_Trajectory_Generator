//
// Created by zheng on 06/05/2021.
//

/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2017 by Hans Joachim Ferreau, Andreas Potschka,
 *	Christian Kirches et al. All rights reserved.
 *
 *	qpOASES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpOASES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpOASES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <memory>
#include <qpOASES.hpp>
#include "robot/panda_mpc.h"
#include "ros/ros.h"
#include <ros/node_handle.h>
#include <panda_mpc/UpdateTrajectoryNextPoint.h>
#include <panda_mpc/trajectoryMsg.h>
#include "ros/time.h"
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <planning/traj_generation.hpp>
#include <robot/robot_mpc_model.h>
#include <sensor_msgs/JointState.h>
#include <robot/robot_model.h>
#include <visualization_msgs/Marker.h>
#include <panda_mpc/trajectoryAcceleration.h>

using namespace planning;

class TrajectSenderNode{

public:
  TrajectSenderNode(ros::NodeHandle & node_handle, Eigen::VectorXd q_init, Eigen::VectorXd qdot_init){

    //=========================== Constructor ====================================
    goal_A_ << 0.391342, 0.843433, 0.49249, -1.51971, -0.468614, 2.24382,   1.03971;

    goal_B_ << -0.391342,0.843433, -0.49249, -1.51971,0.468614, 2.24382, -1.03971;

    init_pos_attend_ = true;
    wait = false;
    execute =true;

    dt_controller_ = 0.001;

    q_in.resize(q_init.size());
    q_des_.resize(q_init.size());
    //====================== Robot Model
    node_handle.getParam("/panda_mpc/root_link_",root_link);
    node_handle.getParam("/panda_mpc/tip_link_",tip_link);

    robot_model_.reset(new robot::RobotModel(node_handle, root_link, tip_link));
    robot_model_->Init(node_handle, q_init, qdot_init);
    dof = robot_model_->getNrOfJoints();

    robot_model_->JntToCart(q_in.q, X_curr_);
    Goal_A_frame_.p[0] = 0.5, Goal_A_frame_.p[1] = 0.5, Goal_A_frame_.p[2] = 0.2;
    Goal_A_frame_.M = X_curr_.M;

    Goal_B_frame_.p[0] = 0.5, Goal_B_frame_.p[1] = -0.5, Goal_B_frame_.p[2] = 0.2;
    Goal_B_frame_.M = X_curr_.M;
    // ===================== Trajectory
    trajectory_generation.reset(new planning::trajGen(node_handle,q_init,qdot_init));

    if(!trajectory_generation->init(node_handle,Goal_A_frame_)){
      ROS_ERROR_STREAM("unable to initialize properly parameters: exit");
    }
    N_ = trajectory_generation->getPredictionNumber();


    q_des_mpc_.resize(N_*dof);
    qd_des_mpc_.resize(N_*dof);
    qdd_des_mpc_.resize(N_*dof);
    solution_.resize(N_*dof);
    solution_.setZero();
    solution_precedent_.resize(N_*dof);
    solution_precedent_.setZero();

    state_.resize(2*dof);
    state_.segment(0,dof) = q_init, state_.segment(dof,dof) = qdot_init;

    state_A_.resize(2*dof, 2*dof);
    state_B_.resize(2*dof,dof);

    q_horizon_.resize(dof*N_);
    qd_horizon_.resize(dof*N_);

    state_A_ = trajectory_generation->getStateA();
    state_B_ = trajectory_generation->getStateB();

    jointState_subscriber = node_handle.subscribe("/panda_mpc/joint_states",1,&TrajectSenderNode::jointStateCallBack,this);
    tip_pos_pub = node_handle.advertise<visualization_msgs::Marker>("/tip_position",1);
    solution_pub = node_handle.advertise<panda_mpc::trajectoryAcceleration>("/mpc_solution",1);
  }

  void jointStateCallBack(const sensor_msgs::JointStatePtr& joint_state );

  bool UpdateMPCTraj();

private:

  ros::Subscriber jointState_subscriber;
  ros::Publisher tip_pos_pub;
  ros::Publisher solution_pub;
  Eigen::Matrix<double,7,1> goal_A_;
  Eigen::Matrix<double,7,1> goal_B_;
  bool init_pos_attend_, execute, wait;
  ros::Time begin_time_, end_time_, wait_begin_, wait_end_;  /*!< @brief duration which control robot plannification module  */

  KDL::Frame X_curr_;
  std::string root_link, tip_link;
  double dt_controller_;
  double dt_mpc_;
  int dof;
  int N_;
  std::shared_ptr<planning::trajGen> trajectory_generation;
  KDL::JntArrayVel q_in;
  KDL::JntArray q_des_; /*!< @brief desired joint position and current joint position */
  Eigen::VectorXd q_des_mpc_, qd_des_mpc_, qdd_des_mpc_ ; /*!<  @brief desired joint parameters in horizon */

  std::shared_ptr<robot::RobotModel> robot_model_;

  KDL::Frame Goal_A_frame_, Goal_B_frame_ , X_mpc_; /*!< @brief goal frame and predicted MPC frame  */
  Eigen::VectorXd solution_, solution_precedent_; /*!<  @brief MPC optimization solution */
  Eigen::MatrixXd state_A_, state_B_;
  Eigen::VectorXd state_; /*!<  @brief state-space model state */
  Eigen::VectorXd q_horizon_, qd_horizon_; /*!<  @brief joint position and velocity in horizon */
  Eigen::Matrix<double,6,7> J;

  int traj_index_;
};

int main(int argc, char** argv )
{
    ros::init(argc,argv, "NextTrajectorySender");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(25);

    Eigen::Matrix<double, 7, 1> q_in, qdot_in;
    q_in << 0.391342, 0.843433, 0.49249, -1.51971, -0.468614, 2.24382,   1.03971;
    qdot_in.setZero();

    TrajectSenderNode trajec_sender(node_handle,q_in,qdot_in);

    while(ros::ok()){

      ros::spinOnce();
      loop_rate.sleep();
    }

    return 0;

}

void TrajectSenderNode::jointStateCallBack(const sensor_msgs::JointStatePtr& joint_state){

  ROS_WARN_STREAM("Calling joint state call back");
  for (int i(0); i < dof; i++){

    q_in.q.data[i] = joint_state->position[i];
    q_in.qdot.data[i] = joint_state->velocity[i];
  }

//  std::cout << "sensor msgs time seq :\n" << joint_state->header.seq << '\n';
//  std::cout << "sensor msgs time stamp :\n" << joint_state->header.stamp << '\n';

  // Update the model
  robot_model_->JntToJac(q_in.q);
  robot_model_->JntToCart(q_in.q, X_curr_);


  // -------------------- Update First MPC trajectory ----------------------------

  Eigen::Vector3d error_task_A, error_task_B;

  error_task_A << X_curr_.p[0] - Goal_A_frame_.p[0], X_curr_.p[1] - Goal_A_frame_.p[1], X_curr_.p[2] - Goal_A_frame_.p[2];
  error_task_B << X_curr_.p[0] - Goal_B_frame_.p[0], X_curr_.p[1] - Goal_B_frame_.p[1], X_curr_.p[2] - Goal_B_frame_.p[2];

//  if(init_pos_attend_ & execute & !wait){

    if(error_task_A.norm()<0.005){
      q_des_.data = goal_B_;
      wait = true;
      wait_begin_ = ros::Time::now();
    }

    if(error_task_B.norm()<0.005){
      q_des_.data = goal_A_;
      wait = true;
      wait_begin_ = ros::Time::now();
    }



    UpdateMPCTraj();
    begin_time_ = ros::Time::now();
    execute = false;
//  }
}

bool TrajectSenderNode::UpdateMPCTraj(){

  traj_index_ = 0;

  for (size_t i(0); i < N_; i++){
    q_des_mpc_.segment(i*dof, dof) = q_des_.data;
  }

  solution_precedent_ = solution_;

  state_.head(dof) =  q_in.q.data;
  state_.tail(dof) = q_in.qdot.data;

  trajectory_generation->getRobotModel()->setJntState(state_.head(dof), state_.tail(dof));

  solution_ = trajectory_generation->update(state_,q_horizon_,qd_horizon_,q_des_mpc_,qd_des_mpc_,solution_precedent_,J);
  q_horizon_ = trajectory_generation->getJointHorizon();
  qd_horizon_ = trajectory_generation->getJointvelHorizon();
  J = robot_model_->getJacobian().data;

//  state_ = state_A_*state_ + state_B_*solution_.head(dof);

  Eigen::Vector3d tip_pos;

  tip_pos << X_curr_.p[0], X_curr_.p[1], X_curr_.p[2];
  uint32_t shape = visualization_msgs::Marker::SPHERE;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "/panda_link0";
  marker.header.stamp = ros::Time::now();
  marker.ns = "obstacle_shape";
  marker.id = 1;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = tip_pos[0];
  marker.pose.position.y = tip_pos[1];
  marker.pose.position.z = tip_pos[2];
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 0.5;
  marker.lifetime = ros::Duration();
  tip_pos_pub.publish(marker);
  ROS_WARN_STREAM("state :\n" << state_.head(dof));

  panda_mpc::trajectoryAcceleration next_acceleration;
  next_acceleration.jntAcc.resize(dof*N_);

  next_acceleration.header.stamp = ros::Time::now();
  for (int i(0); i<dof*N_;i++){
  next_acceleration.jntAcc.at(i) = solution_[i];
  }

  solution_pub.publish(next_acceleration);
  return true;
}
/*
 *	end of file
 */
