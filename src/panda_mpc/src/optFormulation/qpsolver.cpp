#include <optFormulation/qpsolver.h>




QPSolver::qpProblem QPSolver::configureQP(int number_of_variables, int number_of_constraints,qpOASES::Options options)
{
  qpoases_solver->setOptions( options );

  qp_problem.hessian.resize(number_of_variables, number_of_variables);
  qp_problem.gradient.resize(number_of_variables);
  qp_problem.lb.resize(number_of_variables);
  qp_problem.ub.resize(number_of_variables);
  qp_problem.a_constraints.resize(number_of_constraints, number_of_variables);
  qp_problem.lb_a.resize(number_of_constraints);
  qp_problem.ub_a.resize(number_of_constraints);
  qp_problem.solution.resize(number_of_variables);
  qp_problem.nWSR = 1e6;

  return qp_problem;
}

QPSolver::qpProblem QPSolver::configureQP(int number_of_variables, int number_of_constraints)
{
  //--------------------------------------
  // QPOASES
  //--------------------------------------
  qpoases_solver.reset(new qpOASES::SQProblem(number_of_variables,number_of_constraints,qpOASES::HST_POSDEF));

  // QPOases options
  qpOASES::Options options;
  // This options enables regularisation (required) and disable
  // some checks to be very fast !
  // options.setToDefault();
  options.setToFast(); // setToReliable() // setToDefault()
  options.enableRegularisation = qpOASES::BT_FALSE; // since we specify the type of Hessian matrix, we do not need automatic regularisation
  options.enableEqualities = qpOASES::BT_TRUE; // Specifies whether equalities shall be  always treated as active constraints.
  options.printLevel = qpOASES::PL_NONE;
  qpoases_solver->setOptions( options );
  // qpoases_solver->setPrintLevel(qpOASES::PL_NONE); // PL_HIGH for full output, PL_NONE for... none

  return configureQP(number_of_variables,number_of_constraints, options);
}

Eigen::VectorXd QPSolver::SolveQP(QPSolver::qpProblem qp_problem)
{
  if(!qpoases_solver->isInitialised()){
      // Initialise the problem, once it has found a solution, we can hotstart
      ret = qpoases_solver->init(qp_problem.hessian.data(),qp_problem.gradient.data(),qp_problem.a_constraints.data(),qp_problem.lb.data(),qp_problem.ub.data(),qp_problem.lb_a.data(),qp_problem.ub_a.data(),qp_problem.nWSR);
      // Keep init if it didn't work
  }
  else{
      // Otherwise let's reuse the previous solution to find a solution faster
      ret = qpoases_solver->hotstart(qp_problem.hessian.data(),qp_problem.gradient.data(),qp_problem.a_constraints.data(),qp_problem.lb.data(),qp_problem.ub.data(),qp_problem.lb_a.data(),qp_problem.ub_a.data(),qp_problem.nWSR);
  }

  // Zero velocity if no solution found
  qp_problem.solution.setZero();

  if(ret == qpOASES::SUCCESSFUL_RETURN)
      qpoases_solver->getPrimalSolution(qp_problem.solution.data());
  else
     std::cout << "QPOases failed! Sending zero" << std::endl;

  return qp_problem.solution;
}
