#ifndef QPSOLVER_H
#define QPSOLVER_H



#include <Eigen/Dense>
#include <qpOASES.hpp>
#include <boost/scoped_ptr.hpp>
#include <iostream>

class QPSolver
{
public:

    struct qpProblem{
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> hessian;
        Eigen::VectorXd gradient;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> a_constraints;
        Eigen::VectorXd lb;
        Eigen::VectorXd ub;
        Eigen::VectorXd lb_a;
        Eigen::VectorXd ub_a;
        int nWSR;
        Eigen::VectorXd solution;
    };


    QPSolver::qpProblem configureQP(int number_of_variables_, int number_of_constraints_);  /*!< @brief Configure qp with Auctus qpoases options*/
    QPSolver::qpProblem configureQP(int number_of_variables_, int number_of_constraints_, qpOASES::Options options); /*!< @brief Configure qp with specific qpoases options*/

    Eigen::VectorXd SolveQP(QPSolver::qpProblem qpProblem);
private:

    QPSolver::qpProblem qp_problem;

    std::unique_ptr<qpOASES::SQProblem> qpoases_solver; /*!< @brief QP solver point*/

    qpOASES::returnValue ret;
};




#endif // QPSOLVER_H
