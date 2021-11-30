#ifndef CATKIN_WS_TRAJ_GENERATION_H
#define CATKIN_WS_TRAJ_GENERATION_H

#pragma once

#include <iostream>
#include <memory>
#include <qpOASES.hpp>
#include <Eigen/Core>
#include <ros/node_handle.h>
#include <visualization_msgs/MarkerArray.h>

#include <optFormulation/constraint.hpp>
#include <optFormulation/task.hpp>
#include <robot/robot_mpc_model.h>
#include <planning/qpstructure.h>
#include <planning/plane.h>
#include <planning/cubic_spline.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

namespace planning {


class trajGen
{

public:

  trajGen(ros::NodeHandle& node_handle, const Eigen::VectorXd& q_init, const Eigen::VectorXd& qd_init) {

     // Trajectory generation constructor
    dof_ = q_init.size() ;
    q_init_.resize(dof_);
    q_init_ = q_init;
    qd_init_.resize(dof_);
    qd_init_ = qd_init;
    dt_ = 0.05;

    node_handle.getParam("/panda_mpc/N_", N_);
    node_handle.getParam("/panda_mpc/dt_", dt_);
    node_handle.getParam("/panda_mpc/root_link_", root_link_);
    node_handle.getParam("/panda_mpc/tip_link_", tip_link_);

//    human_vertices_subscriber_ = node_handle.subscribe("/hand_centroid_marker",1,&trajGen::humanVerticesCallback,this);
    gru_vertices_subscrier_ = node_handle.subscribe("/hand_markerarray",1,&trajGen::gruVerticesCallback,this);
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
    int reduce_vel = 0.4;
    qd_min_.setConstant(-1.2);
    qd_max_.setConstant(1.2);
    node_handle.getParam("/panda_mpc/qd_min_", qd_min_ros_);
    node_handle.getParam("/panda_mpc/qd_max_", qd_max_ros_);
    ros::param::get("/panda_mpc/robot_member_", robot_member_);
    ros::param::get("/panda_mpc/robot_vertices_", robot_vertices_);
    ros::param::get("/panda_mpc/obstacle_member_", obstacle_member_);
    ros::param::get("/panda_mpc/obstacle_vertices_", obstacle_vertices_);

    q_min_mpc_.resize(dof_*N_);
    q_max_mpc_.resize(dof_*N_);
    qd_min_mpc_.resize(dof_*N_);
    qd_max_mpc_.resize(dof_*N_);

    d_limit_ = 0.4   ;
    d_full_ = 1.5;
    for (size_t i(0); i<N_; i++){
      q_min_mpc_.segment(i*dof_,dof_) = q_min_;
      q_max_mpc_.segment(i*dof_,dof_) = q_max_;
      qd_min_mpc_.segment(i*dof_,dof_) = qd_min_;
      qd_max_mpc_.segment(i*dof_,dof_) = qd_max_;
    }

    // Passive safety constraint
      qd_min_mpc_.tail(dof_).setConstant(-0.05);
      qd_max_mpc_.tail(dof_).setConstant(0.05);

    // Build local MPC trajectory

   cubic_spline_.reset(new cubicSpline::cubicSpline(N_,dt_));


   // ****** Human vertices ---------
   humanVertices_.resize(3, 1);
   humanVerticesAugmented_.resize(1);
   for (std::size_t(j); j < 1; j++){
     humanVerticesAugmented_[j].resize(3,1*N_);
     for (int i(0); i<N_; i++){
          humanVerticesAugmented_[j].block(0,i*1,3,1) << 2, 0, 0;
     }

   }

//   human_vertices_subscriber_ = node_handle.subscribe("/clusters_centroid",1,&trajGen::humanVerticesCallback,this);

  }


  /**
   * @brief Parameters and sub-modules initializers
   * @param node_handle
   * @return
   */
  bool init(ros::NodeHandle& node_handle, KDL::Frame init_frame);

  /**
   * @brief Update MPC solution by solving first the plane QP, then using theses planes to solve locally collision free
   *        trajectory QP optimization problem
   * @param S
   * @param q_horizon
   * @param qd_horizon
   * @param q_des
   * @param qd_des
   * @param solution_precedent
   * @param J
   * @return
   */
  Eigen::VectorXd update(Eigen::VectorXd S, const Eigen::VectorXd &q_horizon, const Eigen::VectorXd & qd_horizon,
                         const Eigen::VectorXd & q_des,const Eigen::VectorXd& qd_des, const Eigen::VectorXd & solution_precedent,
                         Eigen::MatrixXd J);


  /**
   * @brief updateWithAlternatingSolver: the non-linear optimization problem is divided into two QPs, one method to get better
   *        result is to alternating the solution of two QPs while converge.
   * @param S
   * @param q_horizon
   * @param qd_horizon
   * @param q_des
   * @param qd_des
   * @param solution_precedent
   * @param J
   * @return
   */
  Eigen::VectorXd updateWithAlternatingSolver(Eigen::VectorXd S, const Eigen::VectorXd &q_horizon, const Eigen::VectorXd & qd_horizon,
                                              const Eigen::VectorXd & q_des,const Eigen::VectorXd& qd_des, const Eigen::VectorXd & solution_precedent,
                                              Eigen::MatrixXd J);

  void computeSoftMPC(const Eigen::MatrixXd &H, const Eigen::MatrixXd &A,
                      Eigen::VectorXd g,
                      Eigen::VectorXd lbA, Eigen::VectorXd ubA);


  void humanVerticesCallback(visualization_msgs::Marker::Ptr human_position){

    if(human_position == nullptr){
      ROS_WARN_STREAM("There is no person in robot's workspace, robot can go safely");
      for (int i(0); i<N_; i++){
            obsVerticesAugmented_[0].block(0,i*obstacle_vertices_,3,obstacle_vertices_) << 1.5, -0.1, 0.14;
        }
    }else {

//      humanVertices_.block(0,0,3,1) << human_position->markers[0].pose.position.x,
//                                       human_position->markers[0].pose.position.y,
//                                       human_position->markers[0].pose.position.z;
      obsVertices_ << human_position->pose.position.x ,
                      human_position->pose.position.y,
                      human_position->pose.position.z ;
      for (int i(0); i<N_; i++){
            obsVerticesAugmented_[0].block(0,i*obstacle_vertices_,3,obstacle_vertices_) << obsVertices_(0,0), obsVertices_(1,0), obsVertices_(2,0);
        }
    }
  }

  void gruVerticesCallback(const visualization_msgs::MarkerArray::Ptr& gru_pos_array){

    if(gru_pos_array == nullptr){
      ROS_WARN_STREAM("There is no person in robot's workspace, robot can go safely");
      for (int i(0); i<N_; i++){
            obsVerticesAugmented_[0].block(0,i*obstacle_vertices_,3,obstacle_vertices_) << 1.5, -0.1, 0.14;
        }
    }else {


      obsVertices_ << gru_pos_array->markers[0].pose.position.x ,
                      gru_pos_array->markers[0].pose.position.y,
                      gru_pos_array->markers[0].pose.position.z ;

      for (int i(0); i<N_-1; i++){
            obsVerticesAugmented_[0].block(0,i*obstacle_vertices_,3,obstacle_vertices_) << gru_pos_array->markers[i].pose.position.x,
                                                                                           gru_pos_array->markers[i].pose.position.y,
                                                                                           gru_pos_array->markers[i].pose.position.z;
        }
       obsVerticesAugmented_[0].block(0,(N_-1)*obstacle_vertices_,3,obstacle_vertices_) <<gru_pos_array->markers[4].pose.position.x,
                                                                                          gru_pos_array->markers[4].pose.position.y,
                                                                                          gru_pos_array->markers[4].pose.position.z;
    }
  }
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


  bool computeCubicInterpolation(std::vector<KDL::JntArrayAcc> joints, int i){


    return cubic_spline_->JntSpaceCubicInterpolation(joints, i);

  }

  Eigen::MatrixXd getCubicCoefficientMatrix(){
    return cubic_spline_->getCoefficientMatrix();
  }

  int getPredictionNumber(){
    return N_;
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

  // Test with relaxed obstacle avoidance constraint
  qpSolver qpoases_soft_solver_;
  // --------------------------- Plane ---------------------------------------------
  double dsafe_ ;
  double d_limit_, d_full_;

  int robot_member_ ; /*!< @brief number of robot member to be take into account */
  int robot_vertices_ ; /*!< @brief number of robot vertices for one member */
  int obstacle_member_ ; /*!< @brief number of obstacle member to be take into account */
  int obstacle_vertices_ ; /*!< @brief number of obstacle vertices for one member */

  std::unique_ptr<planning::plane> plane_generation; /*!< @brief Computes separating plane */
  ros::Publisher plane_table_pub_, plane_shape_pub_, obstacle_shape_pub_;
  Eigen::MatrixXd robotVertices_;
  std::vector<Eigen::MatrixXd> robotVerticesAugmented_;

  double obs_vel_;
  Eigen::MatrixXd obsVertices_;
  std::vector<Eigen::MatrixXd> obsVerticesAugmented_;

   std::vector<Eigen::MatrixXd> plane_location_, table_plane_location_;
  // Plane private function
  bool publishABCDPlane(double A, double B, double C, double D,
                        double x_width = 2.0, double y_width=2.0 );

  bool publishObstacle(Eigen::Vector3d obstacle_center);

  // --------------------------- Human position from perception node ---------------------
  Eigen::MatrixXd humanVertices_; // First, consider only 1 vertice
  std::vector<Eigen::MatrixXd> humanVerticesAugmented_;

  ros::Subscriber human_vertices_subscriber_, gru_vertices_subscrier_;
  // --------------------------------- MPC trajectory smoothing -------------------------

  std::unique_ptr<cubicSpline::cubicSpline> cubic_spline_;
};

}


#endif
