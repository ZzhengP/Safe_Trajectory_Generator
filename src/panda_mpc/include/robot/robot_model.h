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

class RobotModel{

public:

  RobotModel(ros::NodeHandle &node_handle, const std::string& root_link, const std::string& tip_link):
        root_link_(root_link), tip_link_(tip_link){


      ROS_INFO_STREAM("Constructing robot augemented state for model predictive control");

  }


  /**
   * @brief Initialize robot kinematic and dynamic parameters
   * @param node_handle
   * @return
   */
  bool Init(ros::NodeHandle &node_handle, const Eigen::VectorXd &q_init, const Eigen::VectorXd &qd_init);



  //----------------------------------------------------------------------------------------------
  //                      computation using KDL library
  //----------------------------------------------------------------------------------------------

  void JntToCart(const KDL::JntArray& q_in, KDL::Frame& p_out){

      fksolver_->JntToCart(q_in, p_out);
  }

  void JntToJac(const KDL::JntArray& q_in){

      chainjacsolver_->JntToJac(q_in,J_);
  }

  void JntToJac(const KDL::JntArray& q, KDL::Jacobian& J){

      chainjacsolver_->JntToJac(q,J);
  }

  void CartToJnt(const KDL::Frame& desire_frame, KDL::JntArray& q_sol){
    int i;
    i = ik_solver_->CartToJnt(q_in.q, desire_frame,q_sol);
  }

  void setJntState( Eigen::VectorXd q, Eigen::VectorXd qd){

    q_in.q.data = q;
    q_in.qdot.data = qd;

  }
  //----------------------------------------------------------------------------------------------
  //                      Get robot paramters
  //----------------------------------------------------------------------------------------------
  /**
   * @brief get number of joints of the chain
   * @return
   */
  int getNrOfJoints() const{
      return chain_.getNrOfJoints();
  };

  /**
   * @brief get the Jacobian of tip_link expressed in base frame.
   * @return
   */
  KDL::Jacobian getJacobian() const{
    return J_;
  }

  /**
   * @brief get joint inertial matrix
   * @return
   */
  KDL::JntSpaceInertiaMatrix getJntInertial() const{
    return M_;
  }

  /**
   * @brief get joint lower limit
   * @return
   */
  KDL::JntArray getJntll() const{
    return ll_;
  }

  KDL::JntArray getJntul() const{
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
  KDL::JntArrayVel q_in; /*!< @brief KDL joint position of the robot */

  Eigen::VectorXd x_curr_; /*!< @brief Current robot pose of the robot tip_link */
  Eigen::VectorXd q_init_;
  Eigen::VectorXd qd_init_;
  int dof; /*!< @brief Number of degrees of freedom of the robot */

  std::string root_link_; /*!< @brief base link of the KDL chain */
  std::string tip_link_; /*!< @brief tip link of the KDL chain (usually the end effector*/




};

}
#endif // ROBOT_MODEL_H
