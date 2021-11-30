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
#include <nav_msgs/Path.h>

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
//    0.391342, 0.843433, 0.49249, -1.51971, -0.468614, 2.24382,   1.03971
    goal_A_.resize(7);
    goal_A_.data <<  1.0203, 0.691, -0.2883, -1.589, 0.5157, 2.338, 0.;
// -0.391342,0.843433, -0.49249, -1.51971,0.468614, 2.24382, -1.03971;
    goal_B_.resize(7);
    goal_B_.data << -0.349176, 0.858787,  -0.63727,  -1.36665,  0.600828,  1.81914,  -1.54914;


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
    q_in.q.data =  goal_A_.data;

    robot_model_->JntToCart(q_in.q, X_curr_);
    robot_model_->JntToCart(goal_A_,Goal_A_frame_);
//    Goal_A_frame_.p[0] = 0.5, Goal_A_frame_.p[1] = 0.5, Goal_A_frame_.p[2] = 0.2;
    Goal_A_frame_.M = X_curr_.M;

//    Goal_B_frame_.p[0] = 0.5, Goal_B_frame_.p[1] = -0.5, Goal_B_frame_.p[2] = 0.2;
    robot_model_->JntToCart(goal_B_,Goal_B_frame_);

    Goal_B_frame_.M = X_curr_.M;
    // ===================== Trajectory
    trajectory_generation.reset(new planning::trajGen(node_handle,q_init,qdot_init));

    if(!trajectory_generation->init(node_handle,Goal_A_frame_)){
      ROS_ERROR_STREAM("unable to initialize properly parameters: exit");
    }
    N_ = trajectory_generation->getPredictionNumber();


    q_des_mpc_.resize(N_*dof);
    qd_des_mpc_.resize(N_*dof);
    qd_des_mpc_.setZero();
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


    q_des_ = goal_B_;

    // Comment this line if using real robot
//    jointState_subscriber = node_handle.subscribe("/panda_mpc/joint_states",1,&TrajectSenderNode::jointStateCallBack,this);
    jointState_subscriber = node_handle.subscribe("/franka_state_controller/joint_states",1,&TrajectSenderNode::jointStateCallBack,this);
    solution_pub = node_handle.advertise<panda_mpc::trajectoryAcceleration>("/mpc_solution",1);
    path_pub_ = node_handle.advertise<nav_msgs::Path>("/mpc_path",1);

  }

  void jointStateCallBack(const sensor_msgs::JointStatePtr& joint_state );

  bool UpdateMPCTraj();

  bool getWaitParam(){
    return wait;
  }

  bool publishPath(){

      nav_msgs::Path path;
      path.header.stamp = ros::Time::now();
      path.header.frame_id=  "/panda_link0";


      KDL::JntArray q_in;
      KDL::Frame p_out;

      q_in.resize(dof);
      double q_x, q_y, q_z, q_w;

       X_curr_.M.GetQuaternion(q_x, q_y, q_z, q_w);
      geometry_msgs::PoseStamped current_pose;
      current_pose.pose.position.x =  X_curr_.p[0];
      current_pose.pose.position.y =  X_curr_.p[1];
      current_pose.pose.position.z =  X_curr_.p[2];

      current_pose.pose.orientation.x = q_x;
      current_pose.pose.orientation.y = q_y;
      current_pose.pose.orientation.z = q_z;
      current_pose.pose.orientation.w = q_w;
      current_pose.header.stamp=ros::Time::now();
      current_pose.header.frame_id= "/panda_link0";
      path.poses.push_back(current_pose);

      for (int i(0); i<N_ ;i++){
        q_in.data = q_horizon_.segment(dof*i,dof);
        robot_model_->JntToCart(q_in, p_out);
        KDL::Rotation rot = Goal_A_frame_.M;

        rot.GetQuaternion(q_x, q_y, q_z, q_w);
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x =  p_out.p[0];
        this_pose_stamped.pose.position.y =  p_out.p[1];
        this_pose_stamped.pose.position.z =  p_out.p[2];
        this_pose_stamped.pose.orientation.x = q_x;
        this_pose_stamped.pose.orientation.y = q_y;
        this_pose_stamped.pose.orientation.z = q_z;
        this_pose_stamped.pose.orientation.w = q_w;
        this_pose_stamped.header.stamp=ros::Time::now();
        this_pose_stamped.header.frame_id= "/panda_link0";

         path.poses.push_back(this_pose_stamped);
      }


      path_pub_.publish(path);
  }

private:

  ros::Subscriber jointState_subscriber;
  ros::Publisher solution_pub;
  ros::Publisher path_pub_ ;

  KDL::JntArray goal_A_;
  KDL::JntArray goal_B_;
  bool init_pos_attend_, execute, wait;
  ros::Time begin_time_, end_time_, wait_begin_, wait_end_;  /*!< @brief duration which control robot plannification module  */

  KDL::Frame X_curr_;
  std::string root_link, tip_link;
  double dt_controller_;
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

};

int main(int argc, char** argv )
{
    ros::init(argc,argv, "NextTrajectorySender");
    ros::NodeHandle node_handle;
    double dt;
    node_handle.getParam("/panda_mpc/dt_", dt);

    int rate = 50;
    ros::Rate loop_rate(rate);

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

  for (int i(0); i < dof; i++){
    q_in.q.data[i] = joint_state->position[i];
    q_in.qdot.data[i] = joint_state->velocity[i];
  }


  // Update the model
  robot_model_->JntToJac(q_in.q);
  robot_model_->JntToCart(q_in.q, X_curr_);


  // -------------------- Update First MPC trajectory ----------------------------

  Eigen::Vector3d error_task_A, error_task_B;

  error_task_A << X_curr_.p[0] - Goal_A_frame_.p[0], X_curr_.p[1] - Goal_A_frame_.p[1], X_curr_.p[2] - Goal_A_frame_.p[2];
  error_task_B << X_curr_.p[0] - Goal_B_frame_.p[0], X_curr_.p[1] - Goal_B_frame_.p[1], X_curr_.p[2] - Goal_B_frame_.p[2];


    if(error_task_A.norm()<0.005){

      q_des_ = goal_B_;
      ros::Duration(0.2).sleep();
    }

    if(error_task_B.norm()<0.005){
       q_des_ = goal_A_;
      ros::Duration(0.2).sleep();

   //   wait = true;
    }


    UpdateMPCTraj();

    publishPath();
}

bool TrajectSenderNode::UpdateMPCTraj(){


  for (size_t i(0); i < N_; i++){
    q_des_mpc_.segment(i*dof, dof) = q_des_.data;
  }
  solution_precedent_ = solution_;

  state_.head(dof) =  q_in.q.data;
  state_.tail(dof) = q_in.qdot.data;

  trajectory_generation->getRobotModel()->setJntState(state_.head(dof), state_.tail(dof));
  qd_des_mpc_ = 5*(q_des_mpc_  - q_horizon_) - 4*qd_horizon_;
  qd_des_mpc_.tail(dof).setZero();
  solution_ = trajectory_generation->updateWithAlternatingSolver(state_,q_horizon_,qd_horizon_,q_des_mpc_,qd_des_mpc_,solution_precedent_,J);
//  solution_ = trajectory_generation->update(state_,q_horizon_,qd_horizon_,q_des_mpc_,qd_des_mpc_,solution_precedent_,J);
//  q_horizon_ = trajectory_generation->getJointHorizon();
//  qd_horizon_ = trajectory_generation->getJointvelHorizon();
  J = robot_model_->getJacobian().data;

  q_horizon_ = trajectory_generation->getStatePx()*state_ + trajectory_generation->getStatePu()*solution_;
  qd_horizon_ = trajectory_generation->getStatePxdq()*state_ + trajectory_generation->getStatePudq()*solution_;


//  state_ = state_A_*state_ + state_B_*solution_.head(dof);

  Eigen::Vector3d tip_pos;

  tip_pos << X_curr_.p[0], X_curr_.p[1], X_curr_.p[2];
  uint32_t shape = visualization_msgs::Marker::SPHERE;


  panda_mpc::trajectoryAcceleration next_acceleration;
  next_acceleration.jntAcc.resize(3*dof);


  next_acceleration.header.stamp = ros::Time::now();
  for (int i(0); i<dof;i++){

    next_acceleration.jntAcc.at(i) = solution_[i];
    next_acceleration.jntAcc.at(i+dof) = state_(i);
    next_acceleration.jntAcc.at(i+2*dof) = state_(i+dof);

  }

  solution_pub.publish(next_acceleration);
  return true;
}
/*
 *	end of file
 */
