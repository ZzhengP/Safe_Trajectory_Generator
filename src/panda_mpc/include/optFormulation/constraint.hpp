#ifndef CATKIN_WS_PANDA_CONSTRAINT_H
#define CATKIN_WS_PANDA_CONSTRAINT_H

#pragma once

#include <robot/robot_mpc_model.h>
#include <iostream>
#include <Eigen/Core>
#include <ros/ros.h>
#include <memory>
#include <vector>

namespace optimization {

/**
 * @brief For an quadratic optimization formulation:
 * all constraints are specified as lbA <= Cu <= ubA, and lb <= u <= ub
 */
struct constraint{

  std::string constraint_name_;
  Eigen::VectorXd lbA_;
  Eigen::VectorXd ubA_;
  Eigen::MatrixXd A_;
  int constraint_size_;
};

class MPCConstraint{

public:
  MPCConstraint(const int& N, const int& dof, const double& dt,
                const robot::MPC_param& mpc_params): N_(N), dof_(dof), dt_(dt){

    mpc_params_.init(N_,dof_);
    mpc_params_ = mpc_params;

    constraint_nbr_= 0;
  }

  bool init(const Eigen::VectorXd &q_min, const Eigen::VectorXd &q_max,
            const Eigen::VectorXd &qd_min, const Eigen::VectorXd &qd_max);

  bool update(const Eigen::VectorXd &state);

  void setMPCParams(const robot::MPC_param& param){

      mpc_params_ = param;
  }

  inline void printConstraint(const constraint& constraint){

      std::cout << "-------------------- adding task --------------------- " << '\n';
      std::cout << "name : " << constraint.constraint_name_ << '\n';
      std::cout << "task matrix size: \n" << std::endl;
      std::cout << " A:  " << constraint.A_.rows() << " X " << constraint.A_.cols() << '\n';
      std::cout << " lbA:  " << constraint.lbA_.size() << '\n';
      std::cout << " ubA:  " << constraint.ubA_.size() << '\n';

  }


  void print(){

    for (auto it = constraint_container_.begin(); it!=constraint_container_.end();it++){

      printConstraint(*it);

    }
  }
  // --------------------------------------- Get parama-----------------------
  // Get total constraint number
  int getConstraintNumber() const{

    return constraint_nbr_ ;
  }

  Eigen::VectorXd getLBA() const {
    return lbA_;
  }

  Eigen::VectorXd getUBA() const {
    return ubA_;
  }

  Eigen::MatrixXd getConstraintA() {
    return A_;
  }
private:

  int N_;
  int dof_;
  double dt_;
  Eigen::VectorXd q_min_mpc_;
  Eigen::VectorXd q_max_mpc_;
  Eigen::VectorXd qd_min_mpc_;
  Eigen::VectorXd qd_max_mpc_;

  robot::MPC_param mpc_params_; /*!< @brief model predictive parameters to define mpc task */

  bool updateConstraintContainer(const Eigen::VectorXd& state);

  // ------------------------------------------------------------------------------------------------------------------------
  //                    Quadratic Programming parameters: Cost Function
  //                             lbA <= Au <= ubA
  // ------------------------------------------------------------------------------------------------------------------------
  Eigen::VectorXd lbA_; /*!< @brief lower bound vector of all differents constraints */
  Eigen::VectorXd ubA_; /*!< @brief upper bound vector of all differents constraints */
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_; /*!< @brief constraint matrix of all differents constraints */

  int constraint_nbr_; /*!< @brief the number of all constraints */

  std::vector<constraint> constraint_container_; /*!< @brief vector list of differents constraints */


};

}


#endif
