#ifndef ROBOT_MPC_MODEL_H
#define ROBOT_MPC_MODEL_H

#include <robot/robot_model.h>
#include <iostream>
#include <memory>

namespace robot {

struct MPC_param{

  //------------------------------------ MPC parameters structure for the use in other class------------------


  bool init(const int& N_, const int& dof_){

    A_.resize(2*dof_, 2*dof_);
    A_.setZero();

    B_.resize(2*dof_, dof_);
    B_.setZero();

    Cq_.resize(dof_,2*dof_);
    Cq_.setZero();

    Cdq_.resize(dof_,2*dof_);
    Cdq_.setZero();

    Px_.resize(N_*dof_, 2*dof_);
    Px_.setZero();
    Pu_.resize(N_*dof_, N_*dof_);
    Pu_.setZero();

    Pxdq_.resize(N_*dof_, 2*dof_);
    Pxdq_.setZero();
    Pudq_.resize(N_*dof_, N_*dof_);
    Pudq_.setZero();

    q_horizon_.resize(N_*dof_);
    q_horizon_.setZero();
    qd_horizon_.resize(N_*dof_);
    qd_horizon_.setZero();
    x_horizon_.resize(3*N_);
    J_horizon_.resize(N_*6, N_*dof_);
    J_horizon_.setZero();

    initialized = true;
    return true;
  }

  MPC_param& operator=(const MPC_param& param){

    A_ = param.A_;
    B_ = param.B_;
    Cq_ = param.Cq_;
    Cdq_ = param.Cdq_;
    Px_ = param.Px_;
    Pu_ = param.Pu_;
    Pxdq_ = param.Pxdq_;
    Pudq_ = param.Pudq_;
    q_horizon_ = param.q_horizon_;
    qd_horizon_ = param.qd_horizon_;
    x_horizon_ = param.x_horizon_;
    J_horizon_ = param.J_horizon_;
    initialized = param.initialized;

    return *this;
  }

  // One step state-space dynamic model of a linear system:
  // Xk+1 = A*Xk + B*Uk, with X = (q, qd)

  Eigen::MatrixXd A_; /*! @brief State transition matrix */
  Eigen::MatrixXd B_; /*! @brief State Input matrix */
  Eigen::MatrixXd Cq_; /*! @brief joint poisition Selection matrix: q = Cq_*X */
  Eigen::MatrixXd Cdq_; /*! @brief joint velocity Selection matrix: qd = Cqd_*X */

  Eigen::MatrixXd Px_; /*! @brief augmented linear state matrix for joint position */
  Eigen::MatrixXd Pu_; /*! @brief augmented linear input matrix for joint position*/
  Eigen::MatrixXd Pxdq_; /*! @brief augmented linear state matrix for joint velocity*/
  Eigen::MatrixXd Pudq_; /*! @brief augmented linear input matrix for joint velocity*/

  Eigen::VectorXd q_horizon_; /*! @brief robot joint position stacked for the horizon of prediction  */
  Eigen::VectorXd qd_horizon_;
  Eigen::VectorXd x_horizon_; /*! @brief stacked tip_link cartesian position */
  Eigen::MatrixXd J_horizon_; /*! @brief stacked jacobian */

  bool initialized=false;

};

 // ----------------------------------------------------------------------------------------------------------

 // ------------------------------- Model Predictive Control Robot Model --------------------------------------

class RobotMPcModel:public RobotModel
{
public:
  RobotMPcModel(ros::NodeHandle &node_handle, const std::string& root_link, const std::string& tip_link, const int& N, const double& dt,
                const Eigen::VectorXd &q_init, const Eigen::VectorXd& qd_init)
    :RobotModel{node_handle, root_link,tip_link}, N_{N}, dt_{dt}
  {
     ROS_INFO_STREAM("Robot model predictive control constructor");

     Init(node_handle,q_init,qd_init);

     ROS_INFO_STREAM("Robot model init");


     mpc_params.init(N_, dof);

     Eigen::MatrixXd Id;
     Id.resize(dof, dof);
     Id.setIdentity();
     // Initialize state-space dynamic model

     mpc_params.A_.block(0,0,dof,dof).setIdentity();
     mpc_params.A_.block(0,dof, dof, dof) = Id*dt_;
     mpc_params.A_.block(dof, dof, dof, dof).setIdentity();


     mpc_params.B_.block(0,0, dof, dof)= Id*(dt_*dt_/2);
     mpc_params.B_.block(dof,0,dof, dof)= Id*dt_;


     mpc_params.Cq_.block(0,0,dof,dof).setIdentity();

     mpc_params.Cdq_.block(0,dof,dof,dof).setIdentity();


  }


  /**
   * @brief Initialize model predictive control neccessary variables
   * @param node_handle
   * @return
   */
  bool InitMPCParameter(ros::NodeHandle &node_handle);



  //----------------------------------------------------------------------------------------------
  //                      computation of some variable in the horizon
  //----------------------------------------------------------------------------------------------

  void computeJacobianHorizon(const Eigen::VectorXd & q_horizon);

  // Update MPC parameters
  void update(Eigen::VectorXd state, Eigen::VectorXd solution);


  inline Eigen::MatrixXd matPow(int N, const Eigen::MatrixXd& A){

      Eigen::MatrixXd A_pow;
      A_pow.resize(A.rows(), A.cols());
      A_pow.setIdentity();

      for (int i(0); i<N; i++){

        A_pow = A_pow*A;

      }

      return A_pow;
  }

  inline void setPx(){
    for (int i(0); i<N_;i++)
      {
          Eigen::MatrixXd A;
          A = matPow(i+1,mpc_params.A_);
          mpc_params.Px_.block(dof*i,0,dof,2*dof) = mpc_params.Cq_*A;
      }
  };

  inline void setPu()
  {
      for (int i(0); i<N_; i ++)
      {
          for (int j(0); j<i+1;j++)
          {
              Eigen::MatrixXd A;
              A = matPow(i-j,mpc_params.A_);
              mpc_params.Pu_.block(dof*i,dof*j, dof, dof) = mpc_params.Cq_*A*mpc_params.B_;
          }
      }
  }

  inline void setPxDq()
  {
      for (int i(0); i<N_;i++)
      {
          Eigen::MatrixXd A;
          A = matPow(i+1,mpc_params.A_);
          mpc_params.Pxdq_.block(dof*i,0,dof,2*dof) = mpc_params.Cdq_*A;
      }
  }

  inline void setPuDq(){

    for (int i(0); i<N_; i ++)
    {
        for (int j(0); j<i+1;j++)
        {
            Eigen::MatrixXd A;
            A = matPow(i-j,mpc_params.A_);
            mpc_params.Pudq_.block(dof*i,dof*j,dof,dof) = mpc_params.Cdq_*A*mpc_params.B_;
        }
    }
  }
  //----------------------------------------------------------------------------------------------
  //                      Get MPC variables
  //----------------------------------------------------------------------------------------------

  Eigen::VectorXd getJntHorizon() const {

    return mpc_params.q_horizon_;
  }


  Eigen::VectorXd getJntVelHorizon() const {

    return mpc_params.qd_horizon_;
  }

  Eigen::VectorXd getTipPosHorizon() const {

    return mpc_params.x_horizon_;
  }

  Eigen::MatrixXd getJacobianHorizon() const {

    return mpc_params.J_horizon_;
  }



  MPC_param getMPCParams(){

    return mpc_params;
  }


  Eigen::MatrixXd getStateA(){
    return mpc_params.A_;
  }

  Eigen::MatrixXd getStateB(){
    return mpc_params.B_;
  }

  Eigen::MatrixXd getStatePx(){
    return mpc_params.Px_;
  }

  Eigen::MatrixXd getStatePu(){
    return mpc_params.Pu_;
  }
  Eigen::MatrixXd getStatePxdq(){
    return mpc_params.Pxdq_;
  }

  Eigen::MatrixXd getStatePudq(){
    return mpc_params.Pudq_;
  }

  Eigen::VectorXd getJointHorizon(){
    return mpc_params.q_horizon_;
  }

  Eigen::VectorXd getJointvelHorizon(){
    return mpc_params.qd_horizon_;
  }

private:


  //-------------------------------------------------------------------------------------
  //                                  MPC Parameters
  //-------------------------------------------------------------------------------------
  int N_; /*! @brief lenght of MPC */
  double dt_; /*! @brief sampling time for MPC */


  /**
   * @brief This attribut is declared as a shared pointer
   * because it will be used in task and constraint class
   */
  MPC_param mpc_params ;


};


}

#endif // ROBOT_MPC_MODEL_H
