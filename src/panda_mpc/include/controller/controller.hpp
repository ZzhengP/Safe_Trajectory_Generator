#ifndef CATKIN_WS_CONTROLLER_H
#define CATKIN_WS_CONTROLLER_H

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <controller_interface/multi_interface_controller.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include <panda_mpc/UI.h>
#include <panda_mpc/PandaRunMsg.h>
#include <panda_mpc/UpdateTrajectoryNextPoint.h>
#include <panda_mpc/trajectoryMsg.h>
#include <panda_mpc/trajectoryAcceleration.h>

#include <qpOASES.hpp>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <realtime_tools/realtime_publisher.h>

#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <panda_traj/panda_traj.hpp>
#include <robot/robot_model.h>
#include <planning/traj_generation.hpp>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <planning/cubic_spline.h>

namespace Controller {

class Controller{

public:

  /**
   * @brief bool Init
   * @param nh: ros node handle
   * @param q_init: initial joint position
   * @param qd_init: initial joint velocity
   * @return true if the controller if correctly initialized
   */
  bool Init(ros::NodeHandle& nh, const Eigen::VectorXd &q_init , const Eigen::VectorXd &qd_init );

  bool InitMPCTraj(ros::NodeHandle& nh, const Eigen::VectorXd &q_init , const Eigen::VectorXd &qd_init );
  bool UpdateMPCTraj();

  /**
   * @brief Update for the controller
   * @param q: the current joint position
   * @param dq:  the current joint velocity
   * @param period: the refresh rate of the control
   * @return the desired joint position
   */

  Eigen::VectorXd Update(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const ros::Duration& period);


private:
  template <typename T>
  bool getRosParam(const std::string& param_name,  T& param_data)
  {
    if(!ros::param::get(param_name, param_data))
    {
      ROS_FATAL_STREAM(" Problem loading parameters " << param_name <<". Check the Yaml config file. Killing ros node" );
      ros::shutdown();
    }else
      ROS_INFO_STREAM(param_name << " : " << param_data );
     return true;

  }



  bool getRosParam(const std::string& param_name,  Eigen::VectorXd & param_data)
  {

    std::vector<double> std_param;
    if(!ros::param::get(param_name, std_param))
    {
      ROS_FATAL_STREAM(" Problem loading parameters " << param_name <<". Check the Yaml config file. Killing ros node" );
      ros::shutdown();
    }
    else {
      if (std_param.size() == param_data.size())
      {
        for (int i = 0; i < param_data.size(); ++i)
             param_data(i) = std_param[i];

        ROS_INFO_STREAM(param_name << " : " << param_data );
        return true;

      }
      else
      {
        ROS_FATAL_STREAM(" Wrong matrix size for param " << param_name <<". Check the Yaml config file. Killing ros node");
        ros::shutdown();
      }
    }
  }


  void init_publishers(ros::NodeHandle& nh);

  /**
   * @brief load_paramers: load controller parameters from yaml file
   */
  void load_parameters();

  /**
   * @brief load_robot: load robot from urdf file
   * @param nh
   * @return true if the robot correctly loaded
   */
  bool load_robot(ros::NodeHandle& nh, const Eigen::VectorXd& q_init, const Eigen::VectorXd qd_init);



  /**
   * @brief Publish the trajectory
   */
   void publishTrajectory();

//   void updateTrajectoryPoint(panda_mpc::UpdateTrajectoryNextPoint::Request &req, panda_mpc::UpdateTrajectoryNextPoint::Response &resp);

  void updateTrajectoryPoint(const panda_mpc::trajectoryAcceleration::ConstPtr& traj_acc);


  void computeInterpolationCoefficient();

  // Publishers
  geometry_msgs::Pose X_curr_msg_, X_traj_msg_;
  geometry_msgs::Twist X_err_msg_;
  realtime_tools::RealtimePublisher<geometry_msgs::PoseArray> pose_array_publisher_;
  realtime_tools::RealtimePublisher<nav_msgs::Path> path_publisher_;
  realtime_tools::RealtimePublisher<panda_mpc::PandaRunMsg> panda_rundata_publisher;
  realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> pose_curr_publisher_, pose_des_publisher_;

  // Subscribers
  ros::Subscriber trajectory_msg_subscriber_;

  ros::ServiceServer updateUI_service, updateTraj_service, updateNextTraj_service_;


  std::shared_ptr<robot::RobotModel> robot_model_;

  KDL::Frame X_curr_; /*!< @brief KDL current Cartesian pose of the tip_link */
  KDL::Frame X_traj_; /*!< @brief KDL desired Cartesian pose of the tip_link */
  KDL::Twist Xd_traj_; /*!< @brief KDL desired Cartesian velocity of the tip_link */

  KDL::Twist X_err_; /*!< @brief KDL desired Cartesian error between the desired and current pose */

  KDL::JntArrayVel q_in; /*!< @brief KDL joint position of the robot */
  Eigen::VectorXd p_gains_; /*!< @brief Proportional gains of the PID controller */
  Eigen::VectorXd i_gains_; /*!< @brief Derivative gains of the PID controller */
  Eigen::VectorXd d_gains_; /*!< @brief Integral gains of the PID controller */
  Eigen::VectorXd p_gains_qd_; /*!< @brief Proportional gains of the regularisation controller */
  Eigen::VectorXd joint_velocity_out_; /*!< @brief Results of the QP optimization */
  Eigen::VectorXd joint_velocity_out_precedent_ ;

  Eigen::Matrix<double,6,1> xd_des_; /*!< @brief Desired robot twist of the robot tip_link */
  Eigen::Matrix<double,6,1> x_curr_; /*!< @brief Current robot pose of the robot tip_link */

  double regularisation_weight_; /*!< @brief Regularisation weight */
  int dof; /*!< @brief Number of degrees of freedom of the robot */

  std::string root_link_; /*!< @brief base link of the KDL chain */
  std::string tip_link_; /*!< @brief tip link of the KDL chain (usually the end effector*/

  Eigen::Matrix<double,6,1> x_err; /*!< @brief Cartesian error between the desired and current pose in Eigen */
  Eigen::Matrix<double,6,1> xd_traj_; /*!< @brief desired Cartesian velocity in Eigen */
  Eigen::Matrix <double,6,7> J; /*!< @brief Jacobian in Eigen */
  Eigen::Matrix <double,7,7> M; /*!< @brief Inertia matrix in joint space in Eigen */

  // Matrices for qpOASES
  // NOTE: We need RowMajor (see qpoases doc)
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_; /*!< @brief Hessian matrix of the QP*/
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_; /*!< @brief Constraint matrix of the QP */

  Eigen::VectorXd g_; /*!< @brief Gradient vector of the QP */
  Eigen::VectorXd lb_; /*!< @brief Lower bound vector of the QP */
  Eigen::VectorXd ub_; /*!< @brief Upper bound vector of the QP */
  Eigen::VectorXd lbA_; /*!< @brief Constraint lower bound vector of the QP */
  Eigen::VectorXd ubA_; /*!< @brief Constraint upper bound vector of the QP */
  Eigen::VectorXd qd_min_; /*!< @brief Minimum joint velocity limit vector*/
  Eigen::VectorXd qd_max_; /*!< @brief Maximum joint velocity limit vector*/
  Eigen::VectorXd qdd_max_; /*!< @brief Maximum joint acceleration limit vector*/
  Eigen::VectorXd q_mean_; /*!< @brief Mean joint position of the robot (for the regularization task*/


  std::unique_ptr<qpOASES::SQProblem> qpoases_solver_; /*!< @brief QP solver point*/
  int number_of_constraints_; /*!< @brief Number of constraints of the QP problem*/
  int number_of_variables; /*!< @brief Number of optimization variables of the QP problem*/


  TrajectoryGenerator trajectory; /*!< @brief TrajectoryGenerator object */
  panda_traj::TrajProperties traj_properties_; /*!< @brief Properties of the trajectory */

  KDL::Frame next_tf_;
  geometry_msgs::Vector3 next_point_, preview_point_;
  geometry_msgs::Twist next_vel_;

  panda_mpc::trajectoryMsg trajectory_msg_;


  // ---------------------- Generate MPC trajectory ----------------------------
  std::shared_ptr<planning::trajGen> trajectory_generation; /*!< @brief MPC trajectory generation module */

  bool init_pos_attend_, execute, sub_goal_attend_, wait; /*!< @brief flag used to control task transition */
  int N_; /*!< @brief MPC horizon */
  int traj_index_;
  double dt_; /*!<  @brief MPC sampling time */
  Eigen::VectorXd state_; /*!<  @brief state-space model state */
  Eigen::VectorXd q_horizon_, qd_horizon_; /*!<  @brief joint position and velocity in horizon */
  Eigen::VectorXd solution_, solution_precedent_; /*!<  @brief MPC optimization solution */
  KDL::JntArray q_des_; /*!< @brief desired joint position and current joint position */
  KDL::JntArrayVel q_;
  KDL::JntArrayAcc q_mpc_; /*!< @brief combine joint position and velocity together */

  Eigen::VectorXd ee_vel_; /*!< @brief end-effector cartesian velocity  */
  KDL::Frame Goal_A_frame_, Goal_B_frame_ , X_mpc_; /*!< @brief goal frame and predicted MPC frame  */
  KDL::JntArray q_goal_A_, q_goal_B_;  /*!< @brief two different goals  */

  Eigen::VectorXd jnt_err;
  Eigen::Matrix<double,7,1> jnt_des_;

  std::unique_ptr<cubicSpline::cubicSpline> cubic_spline_;

  std::vector<Eigen::MatrixXd> joints_coefficient_matrix_;/*!< @brief a vector of joint coefficient_matrix */

  std::vector<KDL::JntArrayAcc> joints_array;
};

}


#endif
