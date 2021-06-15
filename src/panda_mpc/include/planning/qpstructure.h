#ifndef QPSTRUCTURE_H
#define QPSTRUCTURE_H

#include <qpOASES.hpp>
#include <Eigen/Core>
#include <memory>
#include <iostream>


struct qpSolver{

  void configureQP(int num_variable, int num_constraint);
  void configureQP(int num_variable, int num_constraint, qpOASES::Options options);
  bool solve();
  void solvePlane();
  // --------------------------- QP solver --------------------------------------
  std::shared_ptr<qpOASES::SQProblem> qp_solver_;
  std::shared_ptr<qpOASES::QProblemB> qp_no_bound_;
  qpOASES::Options options_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_;
  Eigen::VectorXd g_, lb_, ub_, lbA_, ubA_, optimal_solution_;
  int nV_;
  int nbrCst_;
  qpOASES::int_t nWSR_;
  qpOASES::returnValue ret_;


};



#endif // QPSTRUCTURE_H
