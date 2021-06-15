#ifndef CATKIN_WS_TRAJ_GENERATION_H
#define CATKIN_WS_TRAJ_GENERATION_H

#pragma once

#include <iostream>
#include <memory>
#include <qpOASES.hpp>
#include <Eigen/Core>
#include <ros/node_handle.h>

#include <optFormulation/constraint.hpp>
#include <optFormulation/task.hpp>
#include <robot/robot_mpc_model.h>
#include <planning/qpstructure.h>
#include <planning/plane.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

namespace planning {


class trajGen
{

public:

  trajGen(ros::NodeHandle& node_handle, const Eigen::VectorXd& q_init, const Eigen::VectorXd& qd_init){

     // Trajectory generation constructor
    dof_ = q_init.size() ;
    q_init_.resize(dof_);
    q_init_ = q_init;
    qd_init_.resize(dof_);
    qd_init_ = qd_init;
    dt_ = 0.01;

    node_handle.getParam("/panda_mpc/N_", N_);
    node_handle.getParam("/panda_mpc/dt_", dt_);
    node_handle.getParam("/panda_mpc/root_link_", root_link_);
    node_handle.getParam("/panda_mpc/tip_link_", tip_link_);


    // Model
    robot_mpc_model_.reset(new robot::RobotMPcModel(node_handle, root_link_, tip_link_,
                                                    N_,dt_,q_init,qd_init));

    if(!robot_mpc_model_->InitMPCParameter(node_handle)){
      ROS_ERROR_STREAM("Failed to initialize robot Model Predictive Control parameters ");
    }else {
      ROS_INFO_STREAM("Success to initialize robot Model Predictive Control parameters ");
    }

    q_horizon_precedent_.resize(N_*6);

    J_horizon_.resize(N_*6, N_*dof_);

    mpc_param_.init(N_,dof_);
    mpc_param_ = robot_mpc_model_->getMPCParams();

    // Task
    mpc_task_.reset(new optimization::MPCTask(N_,dof_,dt_,mpc_param_ ));

    // Constraint
    mpc_constraint_.reset(new optimization::MPCConstraint(N_,dof_,dt_,mpc_param_));
    q_min_.resize(dof_);
    q_max_.resize(dof_);
    qd_min_.resize(dof_);
    qd_max_.resize(dof_);

    q_min_ = robot_mpc_model_->getJntll().data;
    q_max_ = robot_mpc_model_->getJntul().data;
    qd_min_.setConstant(-4);
    qd_max_.setConstant(4);
    node_handle.getParam("/panda_mpc/qd_min_", qd_min_ros_);
    node_handle.getParam("/panda_mpc/qd_max_", qd_max_ros_);

    q_min_mpc_.resize(dof_*N_);
    q_max_mpc_.resize(dof_*N_);
    qd_min_mpc_.resize(dof_*N_);
    qd_max_mpc_.resize(dof_*N_);

    for (size_t i(0); i<N_; i++){
      q_min_mpc_.segment(i*dof_,dof_) = q_min_;
      q_max_mpc_.segment(i*dof_,dof_) = q_max_;
      qd_min_mpc_.segment(i*dof_,dof_) = qd_min_;
      qd_max_mpc_.segment(i*dof_,dof_) = qd_max_;
    }

    // Passive safety constraint
//      qd_min_mpc_.tail(dof_).setConstant(-1);
//      qd_max_mpc_.tail(dof_).setConstant(+1);

  }


  /**
   * @brief Parameters and sub-modules initializers
   * @param node_handle
   * @return
   */
  bool init(ros::NodeHandle& node_handle, KDL::Frame init_frame);

  Eigen::VectorXd update(Eigen::VectorXd S, const Eigen::VectorXd &q_horizon, const Eigen::VectorXd & qd_horizon,
                         const Eigen::VectorXd & q_des,const Eigen::VectorXd& qd_des, const Eigen::VectorXd & solution_precedent,
                         Eigen::MatrixXd J);

  void setQPdata();

  std::shared_ptr<robot::RobotModel> getRobotModel() {
    return robot_mpc_model_;
  }

  std::shared_ptr<robot::RobotModel> getRobotModel() const{
    return robot_mpc_model_;
  }

  Eigen::MatrixXd getStateA(){
    return robot_mpc_model_->getStateA();
  }

  Eigen::MatrixXd getStateB(){
    return robot_mpc_model_->getStateB();
  }

  Eigen::MatrixXd getStatePx(){
    return robot_mpc_model_->getStatePx();
  }

  Eigen::MatrixXd getStatePu(){
    return robot_mpc_model_->getStatePu();
  }
  Eigen::MatrixXd getStatePxdq(){
    return robot_mpc_model_->getStatePxdq();
  }

  Eigen::MatrixXd getStatePudq(){
    return robot_mpc_model_->getStatePudq();
  }

  Eigen::VectorXd getJointHorizon(){
    return robot_mpc_model_->getJointHorizon();
  }

  Eigen::VectorXd getJointvelHorizon(){
    return robot_mpc_model_->getJointvelHorizon();
  }

  Eigen::MatrixXd computeTipPositionHorizon(const Eigen::VectorXd &q_horizon,
                                            int robot_vertices){
     return robot_mpc_model_->computeTipPositionHorizon(q_horizon,robot_vertices);
  }
private:

  int N_;
  double dt_;
  int dof_;
  std::vector<int> qd_min_ros_, qd_max_ros_;
  Eigen::VectorXd q_min_, q_max_, qd_min_, qd_max_;
  std::string root_link_, tip_link_;
  Eigen::VectorXd q_init_;
  Eigen::VectorXd qd_init_;

  ros::Publisher path_pub_ ;

  // --------------------------- Robot MPC model parameters ----------------------
  std::shared_ptr<robot::RobotMPcModel> robot_mpc_model_;
  Eigen::VectorXd q_horizon_precedent_;
  Eigen::MatrixXd J_horizon_, J_;
  robot::MPC_param mpc_param_;
  // --------------------------- Robot MPC task parameters ----------------------
  std::unique_ptr<optimization::MPCTask> mpc_task_;


  // --------------------------- Robot MPC constraints parameters -----------------
  std::unique_ptr<optimization::MPCConstraint> mpc_constraint_;

  Eigen::VectorXd q_min_mpc_, q_max_mpc_, qd_min_mpc_, qd_max_mpc_;

  // QP solver
  qpSolver qpoases_solver_;
  int nV_;
  int nbrCst_;

  // --------------------------- Plane ---------------------------------------------
  double dsafe_ ;
  int robot_member_ ; /*!< @brief number of robot member to be take into account */
  int robot_vertices_ ; /*!< @brief number of robot vertices for one member */
  int obstacle_member_ ; /*!< @brief number of obstacle member to be take into account */
  int obstacle_vertices_ ; /*!< @brief number of obstacle vertices for one member */

  std::unique_ptr<planning::plane> plane_generation; /*!< @brief Computes separating plane */
  ros::Publisher plane_table_pub_, plane_shape_pub_, obstacle_shape_pub_;
  Eigen::MatrixXd robotVertices_;
  std::vector<Eigen::MatrixXd> robotVerticesAugmented_;

  Eigen::MatrixXd obsVertices_;
  std::vector<Eigen::MatrixXd> obsVerticesAugmented_;

   std::vector<Eigen::MatrixXd> plane_location_, table_plane_location_;
  // Plane private function
  bool publishABCDPlane(double A, double B, double C, double D,
                        double x_width = 2.0, double y_width=2.0 );

  bool publishObstacle(Eigen::Vector3d obstacle_center);

  bool publishPath();

};

}


#endif
