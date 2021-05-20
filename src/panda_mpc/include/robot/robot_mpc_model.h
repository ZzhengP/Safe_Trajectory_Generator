#ifndef ROBOT_MPC_MODEL_H
#define ROBOT_MPC_MODEL_H

#include <robot/robot_model.h>


namespace robot {

class RobotMPcModel:public RobotModel
{
public:
  RobotMPcModel(ros::NodeHandle &node_handle, const std::string& root_link, const std::string& tip_link, const int& N, const double& dt,
                const Eigen::VectorXd &q_init, const Eigen::VectorXd& qd_init)
    :RobotModel{node_handle, root_link,tip_link}, N_{N}, dt_{dt}
  {
     ROS_INFO_STREAM("Robot model predictive control constructor");

     Init(node_handle,q_init,qd_init);
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

  //----------------------------------------------------------------------------------------------
  //                      Get MPC variables
  //----------------------------------------------------------------------------------------------

  Eigen::VectorXd getJntHorizon() const {

    return q_horizon_;
  }


  Eigen::VectorXd getJntVelHorizon() const {

    return qd_horizon_;
  }

  Eigen::VectorXd getTipPosHorizon() const {

    return x_horizon_;
  }

  Eigen::MatrixXd getJacobianHorizon() const {

    return J_horizon_;
  }


private:


  //-------------------------------------------------------------------------------------
  //                                  MPC Parameters
  //-------------------------------------------------------------------------------------
  int N_; /*! @brief lenght of MPC */
  double dt_; /*! @brief sampling time for MPC */

  Eigen::MatrixXd Px_; /*! @brief augmented linear state matrix for joint position */
  Eigen::MatrixXd Pu_; /*! @brief augmented linear input matrix for joint position*/
  Eigen::MatrixXd Pxdq_; /*! @brief augmented linear state matrix for joint velocity*/
  Eigen::MatrixXd Pudq_; /*! @brief augmented linear input matrix for joint velocity*/

  Eigen::VectorXd q_horizon_; /*! @brief robot joint position stacked for the horizon of prediction  */
  Eigen::VectorXd qd_horizon_;
  Eigen::VectorXd x_horizon_; /*! @brief stacked tip_link cartesian position */
  Eigen::MatrixXd J_horizon_; /*! @brief stacked jacobian */


};

}

#endif // ROBOT_MPC_MODEL_H
