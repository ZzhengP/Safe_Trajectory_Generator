#ifndef CATKIN_WS_PANDA_TASK_H
#define CATKIN_WS_PANDA_TASK_H

#pragma once

#include <robot/robot_mpc_model.h>
#include <iostream>
#include <Eigen/Core>
#include <ros/ros.h>
#include <memory>
#include <vector>

using namespace  robot;
namespace optimization {

/**
 * @brief For an quadratic optimization formulation:
 * a task is expressed like a minimization of a quadratic function as ||Ex + f||^2
 */
struct task{

  std::string task_name_;
  Eigen::MatrixXd E_;
  Eigen::VectorXd f_;
  Eigen::VectorXd task_des_;
  double weight;
};


class MPCTask{

public:

  MPCTask(const int& N, const int& dof, const double& dt, std::shared_ptr<MPC_param> mpc_params ): N_(N), dof_(dof), dt_(dt){

    mpc_params_ = std::shared_ptr<robot::MPC_param>(new robot::MPC_param);

    mpc_params_ = mpc_params;

  }

  /**
   * @brief initialize Cost Function parameters
   */
  bool init(const Eigen::VectorXd & q_init);

  bool update(const Eigen::VectorXd& S, const Eigen::VectorXd & q_des, const Eigen::VectorXd& qd_des, const Eigen::VectorXd& qdd_des);

  // ----------------------------------- Get parameters--------------------------------

  /**
   * @brief get a shared pointer of model predictive parameters from robot mpc model
   * @return
   */
  const std::shared_ptr<robot::MPC_param> getMPCParams() {

    return mpc_params_;

  }


  inline void printTask(const task& task){

      std::cout << "-------------------- adding task --------------------- " << '\n';
      std::cout << "name : " << task.task_name_ << '\n';
      std::cout << "task matrix size: \n" << std::endl;
      std::cout << " E:  " << task.E_.rows() << " X " << task.E_.cols() << '\n';
      std::cout << " f:  " << task.f_.size() << '\n';

  }


private:

  int N_; /*!< @brief Number of horizon of prediction */
  int dof_; /*!< @brief robot degree of freedom */
  double dt_; /*!< @brief trajectory generation sampling time */

  std::shared_ptr<robot::MPC_param> mpc_params_; /*!< @brief model predictive parameters to define mpc task */


  bool updateTaskContainer(const Eigen::VectorXd& S, const Eigen::VectorXd & q_des, const Eigen::VectorXd& qd_des, const Eigen::VectorXd& qdd_des);


  // ------------------------------------------------------------------------------------------------------------------------
  //                    Quadratic Programming parameters: Cost Function
  //                             argmin 1/2 x^THx + g^Tx
  // ------------------------------------------------------------------------------------------------------------------------
  Eigen::MatrixXd H_; /*!< @brief Hessian matrix */
  Eigen::VectorXd g_; /*!< @brief gradient vecotr */


  std::vector<task> task_container_; /*!< @brief vector list of differents task */

};

}





#endif
