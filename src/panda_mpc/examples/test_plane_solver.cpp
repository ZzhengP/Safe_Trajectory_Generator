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

using namespace planning;

struct TrajectoryMPC{

  std::vector<Eigen::Matrix<double, 7,1>> joint_position_;
  std::vector<Eigen::Matrix<double, 7,1>> joint_velocity_;
  std::vector<Eigen::Matrix<double, 7,1>> joint_acceleration_;

};

class pointSender
{
public:
  pointSender(ros::NodeHandle &node_handle, Eigen::VectorXd q_init, Eigen::VectorXd qd_init):
              dt{0.001}, node_handle_{node_handle} {




    node_handle_.getParam("/panda_mpc/N_", N);
    node_handle_.getParam("/panda_mpc/dt_", dt);
    node_handle_.getParam("/panda_mpc/root_link_", root_link);
    node_handle_.getParam("/panda_mpc/tip_link_", tip_link);

    dof = 7;
    q_des_mpc.resize(dof*N), qd_des_mpc.resize(dof*N), qdd_des_mpc.resize(dof*N);

    for (size_t i(0); i<N; i++){
      q_des_mpc.segment(i*dof,dof) = q_init;
      qd_des_mpc.segment(i*dof,dof) = qd_init;
    }

    qdd_des_mpc.setZero();


    q_horizon.resize(dof*N);
    qd_horizon.resize(dof*N);
    q_horizon = q_des_mpc;
    qd_horizon = qd_des_mpc;

    state.resize(14);
    state.segment(0,dof) = q_init, state.segment(dof,dof) = qd_init;

    solution.resize(N*dof);
    solution_precedent.resize(N*dof);
    solution.setZero();
    solution_precedent.setZero();

    A.resize(2*dof, 2*dof);
    B.resize(2*dof,dof);

    jnt_error.resize(3);
    jnt_error.setConstant(10);


    x_des.p[0] = 0.5, x_des.p[1] = 0, x_des.p[2] = 0.6;
    x_des.M = x_des.M.Identity();

    q_des.resize(dof);
    q.resize(dof);
    q_in.resize(dof);
    J.resize(dof);
    ee_vel.resize(6);
    trajectory_generation.reset(new planning::trajGen(node_handle,q_init,qd_init));

    trajectory_generation->getRobotModel()->CartToJnt(x_des,q_des);
    if(!trajectory_generation->init(node_handle,x_des)){
      ROS_ERROR_STREAM("Unable to initialize properly parameters: exit");
    }

    for (size_t i(0); i < N; i++){
      q_des_mpc.segment(i*dof, dof) = q_des.data;
    }

    A = trajectory_generation->getStateA();
    B = trajectory_generation->getStateB();

    next_point_client = node_handle.serviceClient<panda_mpc::UpdateTrajectoryNextPoint>("/panda_mpc/next_point");
    robotStateReader_ = node_handle.subscribe("/panda_mpc/joint_states", 1000, &pointSender::update, this);
    trajectory_publisher_ = node_handle.advertise<panda_mpc::trajectoryMsg>("/trajectory_generation/next_point", 1000);
    ROS_WARN_STREAM("constructor finished ");
  }



  void update(const sensor_msgs::JointState::Ptr& joint_state ){

      if(joint_state == nullptr){

        ROS_WARN_STREAM("no joint state information received ");
        return ;
      }else {

      }

      solution_precedent = solution;
      for(int i(0); i < dof ; i++){

        state[i] = joint_state->position[i];
        state[dof+i] = joint_state->velocity[i];
      }

      trajectory_generation->getRobotModel()->setJntState(state.head(dof), state.tail(dof));

      q_horizon = trajectory_generation->getJointHorizon();
      qd_horizon = trajectory_generation->getJointvelHorizon();


      solution = trajectory_generation->update(state,q_horizon,qd_horizon,q_des_mpc,qd_des_mpc,solution_precedent,J.data);

      std::cout <<" solution :\n " << solution.head(dof).transpose() <<'\n';
//      std::cout <<" state :\n" << state.head(dof).transpose() <<'\n';

      state = A*state + B*solution.head(dof);
      q.data = state.head(dof);

      q_in.q.data = state.head(dof);
      q_in.qdot.data = state.tail(dof);

      trajectory_generation->getRobotModel()->JntToCart(q,x_current);
      trajectory_generation->getRobotModel()->JntToJac(q_in.q,J);
      ee_vel = J.data*q_in.qdot.data;

      jnt_error[0] = x_current.p[0] - x_des.p[0];
      jnt_error[1] = x_current.p[1] - x_des.p[1];
      jnt_error[2] = x_current.p[2] - x_des.p[2];



       next_point.x = x_current.p[0];
       next_point.y = x_current.p[1];
       next_point.z = x_current.p[2];

       next_vel.linear.x = ee_vel[0];
       next_vel.linear.y = ee_vel[1];
       next_vel.linear.z = ee_vel[2];
       next_vel.angular.x = ee_vel[3];
       next_vel.angular.y = ee_vel[4];
       next_vel.angular.z = ee_vel[5];

       trajectory_msg_.next_point = next_point;
       trajectory_msg_.next_vel = next_vel;

       trajectory_publisher_.publish(trajectory_msg_);

  }



  /**
   * @brief Test of MPC trajectory planning
   */
  void update(){



    while (jnt_error.norm() > 0.01){
      solution_precedent = solution;



      solution = trajectory_generation->update(state,q_horizon,qd_horizon,q_des_mpc,qd_des_mpc,solution_precedent, J.data);

//      std::cout <<" solution :\n " << solution.head(dof).transpose() <<'\n';
//      std::cout <<" state :\n" << state.head(dof).transpose() <<'\n';
      state = A*state + B*solution.head(dof);
      q.data = state.head(dof);

      q_in.q.data = state.head(dof);
      q_in.qdot.data = state.tail(dof);
      trajectory_generation->getRobotModel()->JntToJac(q_in.q,J);
      ee_vel = J.data*q_in.qdot.data;

      trajectory_generation->getRobotModel()->JntToCart(q,x_current);

      q_horizon = trajectory_generation->getJointHorizon();
      qd_horizon = trajectory_generation->getJointvelHorizon();


      jnt_error[0] = x_current.p[0] - x_des.p[0];
      jnt_error[1] = x_current.p[1] - x_des.p[1];
      jnt_error[2] = x_current.p[2] - x_des.p[2];

      std::cout << "ee position error : \n" << jnt_error.norm() << '\n';


      trajectory_generation->getRobotModel()->setJntState(state.head(dof), state.tail(dof));


      if (jnt_error.norm() < 0.01){
         ROS_INFO("Robot arrives to target position");
         return ;
      }
      }
   }


  void cartToJoint(KDL::Frame kdl_frame, KDL::JntArray & jnt){
    trajectory_generation->getRobotModel()->CartToJnt(kdl_frame, jnt);
  }


  Eigen::MatrixXd computeInterpolation(std::vector<KDL::JntArrayAcc> joints, int i){



    trajectory_generation->computeCubicInterpolation(joints,i);

    return trajectory_generation->getCubicCoefficientMatrix();
  }

private:

  ros::NodeHandle node_handle_;
  ros::ServiceClient next_point_client ;
  ros::Publisher trajectory_publisher_;
  ros::Subscriber robotStateReader_;
  Eigen::Vector3d X_traj_, X_curr_, X_final_ ;
  geometry_msgs::Vector3 next_point;
  geometry_msgs::Twist next_vel;
  panda_mpc::trajectoryMsg trajectory_msg_;

  TrajectoryMPC trajectory_to_send_;

  std::shared_ptr<planning::trajGen> trajectory_generation;

  int N;
  double dt;
  int dof;
  std::string root_link;
  std::string tip_link;
  Eigen::VectorXd state;
  Eigen::VectorXd q_des_mpc, qd_des_mpc, qdd_des_mpc ;
  Eigen::VectorXd q_horizon, qd_horizon;
  Eigen::VectorXd solution, solution_precedent;

  Eigen::MatrixXd A, B;
  Eigen::VectorXd jnt_error;
  KDL::Frame x_des, x_current;
  KDL::JntArray q_des, q;
  TrajectoryMPC trajectory_to_send;
  KDL::Jacobian J;
  KDL::JntArrayVel q_in;
  Eigen::VectorXd ee_vel;

};



int main(int argc, char** argv )
{
    ros::init(argc,argv, "NextTrajectorySender");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(50);


    int dof = 7;
    int N = 4;


    Eigen::VectorXd q_init, qd_init, qdd_init, state;
    q_init.resize(dof), qd_init.resize(dof), qdd_init.resize(dof);
    q_init << 0.0087, -0.1051, 0.0110, -2.279, 0.0018, 2.1754, 0.019;
    qd_init.setZero();
    qdd_init.setZero();


    pointSender point_send_(node_handle, q_init, qd_init);

    KDL::Frame start_fram, end_frame;
    start_fram.p[0] = 0.5, start_fram.p[1] = 0.4, start_fram.p[2] = 0.25;
    end_frame.p[0] = 0.5, end_frame.p[1] = 0., end_frame.p[2] = 0.2;


    start_fram.M.Identity();
    end_frame.M = start_fram.M;

    KDL::Frame frame1,  frame2,  frame3;
    frame1.p[0] = 0.6, frame1.p[1] = 0.4, frame1.p[2] = 0.25;
    frame2.p[0] = 0.6, frame2.p[1] = 0.3, frame2.p[2] = 0.25;
    frame3.p[0] = 0.5, frame3.p[1] = 0.3, frame3.p[2] = 0.25;

    frame1.M = start_fram.M;
    frame2.M = start_fram.M;
    frame3.M = start_fram.M;

    std::vector<KDL::Frame> frames;
    frames.resize(N+1);
    frames[0]= start_fram;
    frames[1] = frame1;
    frames[2] = frame2;
    frames[3] = frame3;
    frames[4] = end_frame;

    std::vector<KDL::JntArrayAcc> local_joints;

    local_joints.resize(N+1);
    for (int i(0); i < N+1; i++){
      std::cout <<" i " << i << '\n';
      local_joints[i].resize(7);
      local_joints[i].q.data = q_init;
      local_joints[i].qdot.data.setConstant(1);
      local_joints[i].qdotdot.data.setConstant(10);

      point_send_.cartToJoint(frames[i], local_joints[i].q);
    }

    Eigen::MatrixXd cubicCoefficient;

    cubicCoefficient.resize(N,4);

    for (int i(0) ; i<7; i++){
    cubicCoefficient = point_send_.computeInterpolation(local_joints,i);


    std::cout << "cubicCoefficient \n" <<  cubicCoefficient << '\n' ;
    }
    ROS_WARN_STREAM("MPC trajectory generated ");





    return 0;

}


/*
 *	end of file
 */
