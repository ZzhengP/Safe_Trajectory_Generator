#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H


#include <memory>
#include <string>
#include <vector>


#include <ros/node_handle.h>
#include <Eigen/Dense>

#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>


namespace robot {

struct robotModelParams
{

};

class robotModel{

public:

  robotModel(ros::NodeHandle &node_handle, const std::string& root_link, const std::string& tip_link):
        root_link_(root_link), tip_link_(tip_link){


      ROS_WARN_STREAM("robot model constructor" );
  }


  /**
   * @brief Initialize robot kinematic and dynamic parameters
   * @param node_handle
   * @return
   */
  bool Init(ros::NodeHandle &node_handle);




  //----------------------------------------------------------------------------------------------
  //                      computation using KDL library
  //----------------------------------------------------------------------------------------------

  void JntToCart(const KDL::JntArray& q_in, KDL::Frame& p_out){

      fksolver_->JntToCart(q_in, p_out);
  }

  void JntToJac(const KDL::JntArray& q_in){
      chainjacsolver_->JntToJac(q_in,J_);
  }
  //----------------------------------------------------------------------------------------------
  //                      Get robot paramters
  //----------------------------------------------------------------------------------------------
  /**
   * @brief get number of joints of the chain
   * @return
   */
  int getNrOfJoints(){
      return chain_.getNrOfJoints();
  };

  /**
   * @brief get the Jacobian of tip_link expressed in base frame.
   * @return
   */
  KDL::Jacobian getJacobian(){
    return J_;
  }

  /**
   * @brief get joint inertial matrix
   * @return
   */
  KDL::JntSpaceInertiaMatrix getJntInertial(){
    return M_;
  }

  /**
   * @brief get joint lower limit
   * @return
   */
  KDL::JntArray getJntll(){
    return ll_;
  }

  KDL::JntArray getJntul(){
    return ul_;
  }
  protected:

  /**
   * @brief Solver for inverse kinematic
   */
  std::shared_ptr<TRAC_IK::TRAC_IK> ik_solver_;



  /**
   * @brief solver to compute Jacobian
   */
  std::unique_ptr<KDL::ChainJntToJacSolver> chainjacsolver_;

  /**
   * @brief solver to compute foward kinematic
   */
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fksolver_;

  std::unique_ptr<KDL::ChainFkSolverVel_recursive> fkvelsolver_;


  KDL::Chain chain_ ; /*! Robot kdl chain */
  KDL::JntArray ll_ ; /*! Joint lower limit */
  KDL::JntArray ul_ ; /*! Joint upper limit */

  KDL::Jacobian J_ ; /*! Jacobian matrix */
  KDL::JntSpaceInertiaMatrix M_; /*! KDL inertia matrix in joint space */


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

};

}
#endif // ROBOT_MODEL_H
