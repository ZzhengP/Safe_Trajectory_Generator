
#include <planning/qpstructure.h>
#include <ros/ros.h>
void qpSolver::configureQP(int num_variable, int num_constraint, qpOASES::Options options){


  qp_solver_->setOptions(options);
  qp_no_bound_->setOptions(options);
  options_ = options;

  H_.resize(num_variable, num_variable);
  g_.resize(num_variable);

  lb_.resize(num_variable);
  ub_.resize(num_variable);

  A_.resize(num_constraint, num_variable);
  lbA_.resize(num_constraint);
  ubA_.resize(num_constraint);

  optimal_solution_.resize(num_variable);
  optimal_solution_.setZero();
  nWSR_ = 1e6;

  nV_ = num_variable;
  nbrCst_ = num_constraint;
}

void qpSolver::configureQP(int num_variable, int num_constraint){

  //---------------------- Reset QP----------------------------------
  qp_solver_.reset(new qpOASES::SQProblem(num_variable, num_constraint, qpOASES::HST_POSDEF));
  qp_no_bound_.reset(new qpOASES::QProblemB(num_variable,qpOASES::HST_POSDEF));
  // Set default options;
  qpOASES::Options options;
  options.setToFast();
  options.enableFlippingBounds = qpOASES::BT_FALSE;
  options.enableRegularisation = qpOASES::BT_FALSE;
  options.enableEqualities = qpOASES::BT_FALSE;
  options.printLevel = qpOASES::PL_HIGH;
  options.numRefinementSteps = 10;

  configureQP(num_variable, num_constraint, options);
}

void qpSolver::configureQPMPC(int num_variable, int num_constraint, qpOASES::Options options){


  qp_solver_mpc_->setOptions(options);
  options_ = options;

  H_.resize(num_variable, num_variable);
  g_.resize(num_variable);

  lb_.resize(num_variable);
  ub_.resize(num_variable);

  A_.resize(num_constraint, num_variable);
  lbA_.resize(num_constraint);
  ubA_.resize(num_constraint);

  optimal_solution_.resize(num_variable);
  optimal_solution_.setZero();
  nWSR_ = 1e6;

  nV_ = num_variable;
  nbrCst_ = num_constraint;
}


void qpSolver::configureQPMPC(int num_variable, int num_constraint){

  //---------------------- Reset QP----------------------------------
  qp_solver_.reset(new qpOASES::SQProblem(num_variable, num_constraint, qpOASES::HST_POSDEF));
  qp_no_bound_.reset(new qpOASES::QProblemB(num_variable,qpOASES::HST_POSDEF));
  qp_solver_mpc_.reset(new qpOASES::QProblem(num_variable, num_constraint, qpOASES::HST_POSDEF));
  // Set default options;
  qpOASES::Options options;
  options.setToFast();
  options.enableFlippingBounds = qpOASES::BT_FALSE;
  options.enableRegularisation = qpOASES::BT_FALSE;
  options.enableEqualities = qpOASES::BT_FALSE;
  options.printLevel = qpOASES::PL_HIGH;
  options.numRefinementSteps = 10;

  configureQPMPC(num_variable, num_constraint, options);
}



bool qpSolver::solve(){

  qp_solver_->setOptions(options_);
  nWSR_ = 1000000;
  if(!qp_solver_->isInitialised()){
//    Initialise the problem, once it has found a solution, we can hotstart
    ret_ = qp_solver_->init(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR_);
  }else
  {
    ret_ = qp_solver_->hotstart(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR_);
  }

  // Zeros acceleration if no solution found
  optimal_solution_.setZero();

  if(ret_ == qpOASES::SUCCESSFUL_RETURN){
    qp_solver_->getPrimalSolution(optimal_solution_.data());
    return true;
  }else{
    std::cout << " print ret " << ": " << qpOASES::returnValue(ret_)<<'\n';
    std::cout << "QPOasese MPC failed! sending zeros solution " << std::endl;
//    std::cout <<"qpProblem status is : " << qp_solver_->getStatus()<<'\n';
//    std::cout <<"qpProblem obj is : " << qp_solver_->getObjVal()<<'\n';
//    std::cout <<"qpProblem cst number is : " << qp_solver_->getNC()<<'\n';
//    std::cout <<"qpProblem variable number is : " << qp_solver_->getNV()<<'\n';
//    std::cout <<"qpProblem is initialized ? " << qp_solver_->isInitialised()<<'\n';
//    std::cout <<"qpProblem is solved ? " << qp_solver_->isSolved()<<'\n';
//    std::cout <<"qpProblem is infeasible ? " << qp_solver_->isInfeasible()<<'\n';
//    std::cout <<"qpProblem is Unbounded ? " << qp_solver_->isUnbounded()<<'\n';
//    std::cout <<" print level : " << qp_solver_->getPrintLevel() << std::endl;
    return false;
  }
}

bool qpSolver::solveMPC(){


  qp_solver_mpc_->setOptions(options_);
  nWSR_ = 1000000;

//    Initialise the problem, once it has found a solution, we can hotstart
    ret_ = qp_solver_mpc_->init(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR_);


  // Zeros acceleration if no solution found
  optimal_solution_.setZero();

  if(ret_ == qpOASES::SUCCESSFUL_RETURN){
    qp_solver_mpc_->getPrimalSolution(optimal_solution_.data());
    return true;
  }else{
    std::cout << " print ret " << ": " << qpOASES::returnValue(ret_)<<'\n';
    std::cout << "QPOasese MPC failed! sending zeros solution " << std::endl;
//        std::cout <<"qpProblem status is : " << qp_solver_->getStatus()<<'\n';
//        std::cout <<"qpProblem obj is : " << qp_solver_->getObjVal()<<'\n';
//        std::cout << "qpProblem hessiantype: " << qp_solver_->getHessianType()<<'\n';
//        std::cout <<"qpProblem cst number is : " << qp_solver_->getNC()<<'\n';
//        std::cout <<"qpProblem variable number is : " << qp_solver_->getNV()<<'\n';
//        std::cout <<"qpProblem is initialized ? " << qp_solver_->isInitialised()<<'\n';
//        std::cout <<"qpProblem is solved ? " << qp_solver_->isSolved()<<'\n';
//        std::cout <<"qpProblem is infeasible ? " << qp_solver_->isInfeasible()<<'\n';
//        std::cout <<"qpProblem is Unbounded ? " << qp_solver_->isUnbounded()<<'\n';
//        std::cout <<" print level : " << qp_solver_->getPrintLevel() << std::endl;
    return false;
  }
}

void qpSolver::configureSoftQPMPC(int num_variable, int num_constraint, qpOASES::Options options){


  qp_soft_solver_mpc_->setOptions(options);
  options_ = options;

  soft_H_.resize(num_variable, num_variable);
  soft_g_.resize(num_variable);

  soft_lb_.resize(num_variable);
  soft_ub_.resize(num_variable);

  soft_A_.resize(num_constraint, num_variable);
  soft_lbA_.resize(num_constraint);
  soft_ubA_.resize(num_constraint);

  soft_optimal_solution_.resize(num_variable);
  soft_optimal_solution_.setZero();
  nWSR_ = 1e6;

  soft_nV_ = num_variable;
  soft_nbrCst_ = num_constraint;
}


void qpSolver::configureSoftQPMPC(int num_variable, int num_constraint){

  //---------------------- Reset QP----------------------------------
  qp_soft_solver_mpc_.reset(new qpOASES::SQProblem(num_variable, num_constraint, qpOASES::HST_SEMIDEF));
  // Set default options;
  qpOASES::Options options;
  options.setToFast();
  options.enableFlippingBounds = qpOASES::BT_FALSE;
  options.enableRegularisation = qpOASES::BT_FALSE;
  options.enableEqualities = qpOASES::BT_FALSE;
  options.printLevel = qpOASES::PL_HIGH;
  options.numRefinementSteps = 10;

  configureSoftQPMPC(num_variable, num_constraint, options);
}

bool qpSolver::solveSoftMPC(){


  qp_soft_solver_mpc_->setOptions(options_);
  nWSR_ = 1000000;
//    Initialise the problem, once it has found a solution, we can hotstart
  static bool qpoases_initialized = false;
  if(!qpoases_initialized){
//    Initialise the problem, once it has found a solution, we can hotstart
    ret_ = qp_soft_solver_mpc_->init(soft_H_.data(),soft_g_.data(),soft_A_.data(),soft_lb_.data(),soft_ub_.data(),soft_lbA_.data(),soft_ubA_.data(),nWSR_);

    // Keep init if it didn't work
    if(ret_ == qpOASES::SUCCESSFUL_RETURN)
    qpoases_initialized = true;
  }else
  {
    ret_ = qp_soft_solver_mpc_->hotstart(soft_H_.data(),soft_g_.data(),soft_A_.data(),soft_lb_.data(),soft_ub_.data(),soft_lbA_.data(),soft_ubA_.data(),nWSR_);
  if(ret_ != qpOASES::SUCCESSFUL_RETURN)
      qpoases_initialized = false;
  }
  // Zeros acceleration if no solution found
  soft_optimal_solution_.setZero();

  if(ret_ == qpOASES::SUCCESSFUL_RETURN){
    qp_soft_solver_mpc_->getPrimalSolution(soft_optimal_solution_.data());

    return true;
  }else{
    std::cout << " print ret " << ": " << qpOASES::returnValue(ret_)<<'\n';
    ROS_WARN_STREAM("QPOasese soft MPC failed! sending zeros solution");
    std::cout <<"qpProblem status is : " << qp_soft_solver_mpc_->getStatus()<<'\n';
    std::cout <<"qpProblem obj is : " << qp_soft_solver_mpc_->getObjVal()<<'\n';
    std::cout << "qpProblem hessiantype: " << qp_soft_solver_mpc_->getHessianType()<<'\n';
    std::cout <<"qpProblem cst number is : " << qp_soft_solver_mpc_->getNC()<<'\n';
    std::cout <<"qpProblem variable number is : " << qp_soft_solver_mpc_->getNV()<<'\n';
    std::cout <<"qpProblem is initialized ? " << qp_soft_solver_mpc_->isInitialised()<<'\n';
    std::cout <<"qpProblem is solved ? " << qp_soft_solver_mpc_->isSolved()<<'\n';
    std::cout <<"qpProblem is infeasible ? " << qp_soft_solver_mpc_->isInfeasible()<<'\n';
    std::cout <<"qpProblem is Unbounded ? " << qp_soft_solver_mpc_->isUnbounded()<<'\n';
    std::cout <<" print level : " << qp_soft_solver_mpc_->getPrintLevel() << std::endl;
    return false;
  }
}
