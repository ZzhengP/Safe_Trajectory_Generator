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
#include "robot/panda_controller.h"
#include "ros/ros.h"
#include <ros/node_handle.h>
#include <panda_mpc/UpdateTrajectoryNextPoint.h>
#include "ros/time.h"
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <robot/robot_mpc_model.h>
#include <optFormulation/task.hpp>


using namespace robot;

class pointSender
{
public:
  pointSender(ros::NodeHandle &node_handle): period_(0.05) {
    next_point_client = node_handle.serviceClient<panda_mpc::UpdateTrajectoryNextPoint>("/panda_mpc/next_point");
    EEPositionReader_ = node_handle.subscribe("/panda_mpc/ee_pose", 1, &pointSender::readEndEffectorPos, this);
    X_traj_ << 0.5, 0 , 0.4;
    X_final_ << 0.5, 0.3, 0.4;
    next_point_.linear.x = 0.5,  next_point_.linear.y = 0 ,  next_point_.linear.z = 0.4;
    next_point_.angular.x = 0, next_point_.angular.y = 3.15149, next_point_.angular.z = 3.15149 ;
  }


  void readEndEffectorPos(const geometry_msgs::PoseStamped::Ptr& ee_pos ){


      X_curr_ << ee_pos->pose.position.x, ee_pos->pose.position.y, ee_pos->pose.position.z ;

      std::cout << " End effector current position :\n" << X_curr_ << '\n';

      double error, goal_error;
      error = (X_curr_ - X_traj_).norm();
      goal_error = (X_curr_ - X_final_).norm();

      ROS_WARN_STREAM("target error " << goal_error);
      ROS_WARN_STREAM("trajectory error " << error);

      if (goal_error < 0.05){
         ROS_INFO("Robot arrives to target position");
      }else {
        if (error < 0.02){
           ROS_INFO("Robot arrives to intermediate position: send next position");
           double x, y,z;
           x = X_curr_[0], y = X_curr_[1] + 0.01 , z = X_curr_[2];

           next_point_.linear.x = x,  next_point_.linear.y = y ,  next_point_.linear.z = z;
           X_traj_ << x, y , z;
           panda_mpc::UpdateTrajectoryNextPoint next_point_srv;
           next_point_srv.request.next_point = next_point_;
           next_point_srv.request.vel = 0.1;
           next_point_srv.response.success = true;

           next_point_client.call(next_point_srv);
        }else
        {
          X_traj_ << next_point_.linear.x, next_point_.linear.y, next_point_.linear.z;
          panda_mpc::UpdateTrajectoryNextPoint next_point_srv;
          next_point_srv.request.next_point = next_point_;
          next_point_srv.request.vel = 0.1;
          next_point_srv.response.success = true;

           next_point_client.call(next_point_srv);
        }
      }
  }

private:
  const double period_;
  ros::ServiceClient next_point_client ;
  ros::Subscriber EEPositionReader_;
  Eigen::Vector3d X_traj_, X_curr_, X_final_ ;
  geometry_msgs::Twist next_point_;

};



int main(int argc, char** argv )
{
    ros::init(argc,argv, "NextTrajectorySender");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(10);

    int N = 1;
    double dt = 0.05;
    std::string root_link;
    std::string tip_link;

    node_handle.getParam("/panda_mpc/N_", N);
    node_handle.getParam("/panda_mpc/dt_", dt);
    node_handle.getParam("/panda_mpc/root_link_", root_link);
    node_handle.getParam("/panda_mpc/tip_link_", tip_link);

    Eigen::VectorXd q_init, qd_init, qdd_init, state;
    q_init.resize(7), qd_init.resize(7), qdd_init.resize(7);
    q_init << 0.0087, -0.1051, 0.0110, -2.279, 0.0018, 2.1754, 0.019;
    qd_init.setZero();
    qdd_init.setZero();
    state.resize(14);
    state.segment(0,7) = q_init, state.segment(7,7) = qd_init;
    Eigen::VectorXd q_des_horizon, qd_des_horizon, qdd_des_horizon ;
    q_des_horizon.resize(7*N), qd_des_horizon.resize(7*N), qdd_des_horizon.resize(7*N);
    for (size_t i(0); i<N; i++){
      q_des_horizon.segment(i*7,7) = q_init;
      qd_des_horizon.segment(i*7,7) = qd_init;
      qdd_des_horizon.segment(i*7,7) = qdd_init;


    }
//=======================================================
//   pointSender pointsender(node_handle);

//    while (ros::ok()){




//      ros::spinOnce();
//      loop_rate.sleep();
//    }
//=======================================================

    std::shared_ptr<robot::RobotMPcModel> robot_mpc_model;

    robot_mpc_model.reset(new robot::RobotMPcModel(node_handle,root_link,tip_link,N,dt,q_init,qd_init));

    if(!robot_mpc_model->InitMPCParameter(node_handle)){
      ROS_ERROR_STREAM("Failed to initialize Model Predictive Control ");
    }else {
      ROS_INFO_STREAM("Success to initialize Model Predictive Control ");
    }

    std::unique_ptr<optimization::MPCTask> mpc_task;
    mpc_task.reset(new optimization::MPCTask(N,7,dt,robot_mpc_model->getMPCParams()));

    if(!mpc_task->init(q_init)){
      return 0;
    };

    mpc_task->update(state,q_des_horizon,qd_des_horizon, qdd_des_horizon);


    return 0;

}


/*
 *	end of file
 */
