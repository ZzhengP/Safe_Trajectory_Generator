#include <planning/traj_generation.hpp>

namespace planning {

  bool trajGen::init(ros::NodeHandle& node_handle){


    if(!mpc_task_->init(q_init_)){
      ROS_ERROR_STREAM("Failed to initialize MPC task parameters ");
      return  false;
     }else {
      ROS_WARN_STREAM("Success to initialize MPC task parameters ");
      mpc_task_->print();

     }


    if(!mpc_constraint_->init(q_min_mpc_,q_max_mpc_,qd_min_mpc_,qd_max_mpc_)){
      ROS_ERROR_STREAM("Failed to initialize MPC constraints parameters ");
      return false;
    }else{
      ROS_WARN_STREAM("Success to initialize MPC constraints parameters ");
      mpc_constraint_->print();
    }

    // qp_oases solver init
    nV_ = N_*dof_;
    nbrCst_ = mpc_constraint_->getConstraintNumber();

    qpoases_solver_.configureQP(nV_,nbrCst_);
    return true;
  }



  Eigen::VectorXd trajGen::update(Eigen::VectorXd S, const Eigen::VectorXd &q_horizon, const Eigen::VectorXd & qd_horizon,
                                  const Eigen::VectorXd &q_des, const Eigen::VectorXd &qd_des,const Eigen::VectorXd &solution_precedent){


    Eigen::VectorXd qdd_des(N_*dof_);
    qdd_des.setZero();

    // update robot model
    robot_mpc_model_->setJntState(S.segment(0,dof_), S.segment(dof_,dof_));
    robot_mpc_model_->update(S,solution_precedent,robot_mpc_model_->getJointHorizon());

    J_horizon_ = robot_mpc_model_->getMPCParams().J_horizon_;

    // Update robot desired task
    if(!mpc_task_->update(S, q_des,qd_des,qdd_des)){
      ROS_INFO_STREAM("failed to update task");
    }else{
      ROS_INFO_STREAM("success to update task");

    }
    qpoases_solver_.H_ = mpc_task_->getHessien();
    qpoases_solver_.g_ = mpc_task_->getGradient();

    // Update constraint
    if(!mpc_constraint_->update(S)){
      ROS_INFO_STREAM("failed to update constraint");
    }
    else{
          ROS_INFO_STREAM("success to update constraint");
    }
    qpoases_solver_.lb_.setConstant(-10);
    qpoases_solver_.ub_.setConstant(10);
    qpoases_solver_.lbA_ = mpc_constraint_->getLBA();
    qpoases_solver_.ubA_ = mpc_constraint_->getUBA();
    qpoases_solver_.A_ = mpc_constraint_->getConstraintA();

    qpoases_solver_.solve();
    return qpoases_solver_.optimal_solution_;
  }



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
    options.enableRegularisation = qpOASES::BT_FALSE;
    options.enableEqualities = qpOASES::BT_TRUE;
    options.printLevel = qpOASES::PL_NONE;

    configureQP(num_variable, num_constraint, options);
  }


  void qpSolver::solve(){

    qp_solver_->setOptions(options_);
    nWSR_ = 1000000;
    if(!qp_solver_->isInitialised()){
      //Initialise the problem, once it has found a solution, we can hotstart
      ret_ = qp_solver_->init(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR_);
    }else
    {
      ret_ = qp_solver_->hotstart(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR_);
    }

    // Zeros acceleration if no solution found
    optimal_solution_.setZero();

    if(ret_ == qpOASES::SUCCESSFUL_RETURN){
      qp_solver_->getPrimalSolution(optimal_solution_.data());
    }else{
      std::cout << "QPOasese failed! sending zeros solution " << std::endl;
    }
  }


  enum  	returnValue {
    TERMINAL_LIST_ELEMENT = -1, SUCCESSFUL_RETURN = 0, RET_DIV_BY_ZERO, RET_INDEX_OUT_OF_BOUNDS,
    RET_INVALID_ARGUMENTS, RET_ERROR_UNDEFINED, RET_WARNING_UNDEFINED, RET_INFO_UNDEFINED,
    RET_EWI_UNDEFINED, RET_AVAILABLE_WITH_LINUX_ONLY, RET_UNKNOWN_BUG, RET_PRINTLEVEL_CHANGED=10,
    RET_NOT_YET_IMPLEMENTED, RET_INDEXLIST_MUST_BE_REORDERD, RET_INDEXLIST_EXCEEDS_MAX_LENGTH, RET_INDEXLIST_CORRUPTED,
    RET_INDEXLIST_OUTOFBOUNDS, RET_INDEXLIST_ADD_FAILED, RET_INDEXLIST_INTERSECT_FAILED, RET_INDEX_ALREADY_OF_DESIRED_STATUS,
    RET_ADDINDEX_FAILED, RET_REMOVEINDEX_FAILED=20, RET_SWAPINDEX_FAILED, RET_NOTHING_TO_DO,
    RET_SETUP_BOUND_FAILED, RET_SETUP_CONSTRAINT_FAILED, RET_MOVING_BOUND_FAILED, RET_MOVING_CONSTRAINT_FAILED,
    RET_SHIFTING_FAILED, RET_ROTATING_FAILED, RET_QPOBJECT_NOT_SETUP, RET_QP_ALREADY_INITIALISED=30,
    RET_NO_INIT_WITH_STANDARD_SOLVER, RET_RESET_FAILED, RET_INIT_FAILED, RET_INIT_FAILED_TQ,
    RET_INIT_FAILED_CHOLESKY, RET_INIT_FAILED_HOTSTART, RET_INIT_FAILED_INFEASIBILITY, RET_INIT_FAILED_UNBOUNDEDNESS,
    RET_INIT_FAILED_REGULARISATION, RET_INIT_SUCCESSFUL=40, RET_OBTAINING_WORKINGSET_FAILED, RET_SETUP_WORKINGSET_FAILED,
    RET_SETUP_AUXILIARYQP_FAILED, RET_NO_CHOLESKY_WITH_INITIAL_GUESS, RET_NO_EXTERN_SOLVER, RET_QP_UNBOUNDED,
    RET_QP_INFEASIBLE, RET_QP_NOT_SOLVED, RET_QP_SOLVED, RET_UNABLE_TO_SOLVE_QP=50,
    RET_INITIALISATION_STARTED, RET_HOTSTART_FAILED, RET_HOTSTART_FAILED_TO_INIT, RET_HOTSTART_FAILED_AS_QP_NOT_INITIALISED,
    RET_ITERATION_STARTED, RET_SHIFT_DETERMINATION_FAILED, RET_STEPDIRECTION_DETERMINATION_FAILED, RET_STEPLENGTH_DETERMINATION_FAILED,
    RET_OPTIMAL_SOLUTION_FOUND, RET_HOMOTOPY_STEP_FAILED=60, RET_HOTSTART_STOPPED_INFEASIBILITY, RET_HOTSTART_STOPPED_UNBOUNDEDNESS,
    RET_WORKINGSET_UPDATE_FAILED, RET_MAX_NWSR_REACHED, RET_CONSTRAINTS_NOT_SPECIFIED, RET_INVALID_FACTORISATION_FLAG,
    RET_UNABLE_TO_SAVE_QPDATA, RET_STEPDIRECTION_FAILED_TQ, RET_STEPDIRECTION_FAILED_CHOLESKY, RET_CYCLING_DETECTED=70,
    RET_CYCLING_NOT_RESOLVED, RET_CYCLING_RESOLVED, RET_STEPSIZE, RET_STEPSIZE_NONPOSITIVE,
    RET_SETUPSUBJECTTOTYPE_FAILED, RET_ADDCONSTRAINT_FAILED, RET_ADDCONSTRAINT_FAILED_INFEASIBILITY, RET_ADDBOUND_FAILED,
    RET_ADDBOUND_FAILED_INFEASIBILITY, RET_REMOVECONSTRAINT_FAILED, RET_REMOVEBOUND_FAILED, RET_REMOVE_FROM_ACTIVESET,
    RET_ADD_TO_ACTIVESET, RET_REMOVE_FROM_ACTIVESET_FAILED, RET_ADD_TO_ACTIVESET_FAILED, RET_CONSTRAINT_ALREADY_ACTIVE,
    RET_ALL_CONSTRAINTS_ACTIVE, RET_LINEARLY_DEPENDENT, RET_LINEARLY_INDEPENDENT, RET_LI_RESOLVED,
    RET_ENSURELI_FAILED, RET_ENSURELI_FAILED_TQ, RET_ENSURELI_FAILED_NOINDEX, RET_ENSURELI_FAILED_CYCLING,
    RET_BOUND_ALREADY_ACTIVE, RET_ALL_BOUNDS_ACTIVE, RET_CONSTRAINT_NOT_ACTIVE, RET_BOUND_NOT_ACTIVE,
    RET_HESSIAN_NOT_SPD, RET_HESSIAN_INDEFINITE, RET_MATRIX_SHIFT_FAILED, RET_MATRIX_FACTORISATION_FAILED,
    RET_PRINT_ITERATION_FAILED, RET_NO_GLOBAL_MESSAGE_OUTPUTFILE, RET_DISABLECONSTRAINTS_FAILED, RET_ENABLECONSTRAINTS_FAILED,
    RET_ALREADY_ENABLED, RET_ALREADY_DISABLED, RET_NO_HESSIAN_SPECIFIED, RET_USING_REGULARISATION,
    RET_EPS_MUST_BE_POSITVE, RET_REGSTEPS_MUST_BE_POSITVE, RET_HESSIAN_ALREADY_REGULARISED, RET_CANNOT_REGULARISE_IDENTITY,
    RET_CANNOT_REGULARISE_SPARSE, RET_NO_REGSTEP_NWSR, RET_FEWER_REGSTEPS_NWSR, RET_CHOLESKY_OF_ZERO_HESSIAN,
    RET_ZERO_HESSIAN_ASSUMED, RET_CONSTRAINTS_ARE_NOT_SCALED, RET_INITIAL_BOUNDS_STATUS_NYI, RET_ERROR_IN_CONSTRAINTPRODUCT,
    RET_FIX_BOUNDS_FOR_LP, RET_USE_REGULARISATION_FOR_LP, RET_UPDATEMATRICES_FAILED, RET_UPDATEMATRICES_FAILED_AS_QP_NOT_SOLVED,
    RET_UNABLE_TO_OPEN_FILE, RET_UNABLE_TO_WRITE_FILE, RET_UNABLE_TO_READ_FILE, RET_FILEDATA_INCONSISTENT,
    RET_OPTIONS_ADJUSTED, RET_UNABLE_TO_ANALYSE_QPROBLEM, RET_NWSR_SET_TO_ONE, RET_UNABLE_TO_READ_BENCHMARK,
    RET_BENCHMARK_ABORTED, RET_INITIAL_QP_SOLVED, RET_QP_SOLUTION_STARTED, RET_BENCHMARK_SUCCESSFUL,
    RET_NO_DIAGONAL_AVAILABLE, RET_DIAGONAL_NOT_INITIALISED, RET_ENSURELI_DROPPED, RET_SIMPLE_STATUS_P1,
    RET_SIMPLE_STATUS_P0, RET_SIMPLE_STATUS_M1, RET_SIMPLE_STATUS_M2, RET_SIMPLE_STATUS_M3
  };



}






