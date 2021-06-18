
#include <planning/plane_solver_old.h>

PlaneSolver::PlaneSolver(int nbrCst, double dsafe):nC_(nbrCst), dsafe_(dsafe)
{
    nV_ = 5; // wo cao, wo ri le  !!!!!
    options_.enableFlippingBounds = qpOASES::BT_FALSE;
    options_.enableRegularisation = qpOASES::BT_FALSE; // since we specify the type of Hessian matrix, we do not need automatic regularisation
    options_.enableEqualities = qpOASES::BT_TRUE; //
    options_.numRefinementSteps = 10;
    data_optimal_solution_.resize(nV_);

    lb_.resize(nV_);
    ub_.resize(nV_);
    lb_ << -1, -1, -1, 0.0, 0;
    ub_ << 1, 1, 0, 10, 0.5;


    E_.resize(5,5);
    E_.setZero();
    E_.block(0,0,4,4).setIdentity();
    H_.resize(5,5);
//    H_ = E_.transpose() * E_ ;
//    H_.setIdentity();
    g_.resize(5);
}



void PlaneSolver::setCost(const Eigen::VectorXd& f){
    Eigen::VectorXd vecI;

//    g_.setZero();
    Eigen::MatrixXd H1,H2,H3,E1, H4;
    Eigen::VectorXd g1,g2,g3,g4;
    H1.resize(5,5);
    H2.resize(5,5);
    H3.resize(5,5);
    g1.resize(5);
    g2.resize(5);
    g3.resize(5);

    E1.resize(4,5);
    E1.setZero();
    E1.block(0,0,4,4).setIdentity();
    H1 = E1.transpose()*E1;
    g1 = - E1.transpose()*f.segment(0,4);


    // plane interpenetration problem
    H2.setZero();
    g2 << 0,0,0,0,1;

    H3.setZero();
    H3(4,4) = 0.2;
    g3.setZero();




    // For non linear
    H_ = 0.1*H1 + H2 + 0.1*H3;
    g_ = 0.1*g1 + g2 + 0.1*g3;
}


void PlaneSolver::setCstMatrix(const Eigen::MatrixXd &robotPartielVertices,
                               const Eigen::MatrixXd &obsPartielVertices,
                               const Eigen::MatrixXd &dataPlanePrecedent){

  int rcols = robotPartielVertices.cols(), pcols = obsPartielVertices.cols();
   A_.resize(rcols + pcols +1, 5);

  //  Additional non vertical plane:
  A_.setZero();
  //  min -d + alpha*d² + ||(ak,bk) - (ak,bk)^p||²
  // dsafe  << -rk.ak + bk
  // dsafe << pk.ak - bk - dk
  // 1-e << ak^p.ak << 1
  // Plane close to robot


   lbA_.resize(rcols+pcols+1);
   ubA_.resize(rcols+pcols+1);
   lbA_.setZero();

  for (int j(0); j < rcols ; j ++ ){
      A_.block(j,0,1,5) << -robotPartielVertices.block(0,j,3,1).transpose() , 1, 1;
  }
  for (int i(0); i < pcols; i++ ){
      A_.block(i+rcols,0,1,5) << obsPartielVertices.block(0,i,3,1).transpose(), -1, 1;
  }



  A_.block(rcols + pcols,0,1,5) << dataPlanePrecedent(0),dataPlanePrecedent(1),dataPlanePrecedent(2),0,0 ;
  // Additional projection constraint to keep plane with an angle


  // Additional non vertical constraint

  dsafe_ = 0.1;
  lbA_.setConstant(dsafe_);
  lbA_(rcols + pcols) = 0.95;


  ubA_.setConstant(10000000);
  ubA_(0) = 1;
  ubA_(1) = 1;
  // ubA_(rcols + pcols +1) = 1;
  ubA_(rcols + pcols) = 1;

}


bool PlaneSolver::solve(){
    qpOASES::returnValue ret;
    qpOASES::PrintLevel printlevel;
    printlevel = qpOASES::PL_HIGH;
    qpOASES::int_t nWSR = 100000;
//     if(!qpoases_initialized)
//           {
        // Initialise the problem, once it has found a solution, we can hotstart
    int cstNbr;
    cstNbr = lbA_.size();

//    qpOASES::QProblem example(5,cstNbr);
//    qpOASES::Options options_;
//    options_.numRefinementSteps = 10;
//    options_.setToMPC();
//    example.setPrintLevel(qpOASES::PL_HIGH);
//    example.setOptions(options_);

    qpOASES::SQProblem example(5,cstNbr,qpOASES::HST_POSDEF);
    qpOASES::Options options_;
    options_.numRefinementSteps=100;
    options_.setToReliable();
    options_.enableRegularisation = qpOASES::BT_FALSE; // since we specify the type of Hessian matrix, we do not need automatic regularisation
    options_.enableEqualities = qpOASES::BT_FALSE; // Specifies whether equalities shall be  always treated as active constraints.
    example.setOptions(options_);
    example.setPrintLevel(qpOASES::PL_HIGH);

    if(!example.isInitialised()){
      ret = example.init( H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(), nWSR );

    }else {

      ret = example.hotstart(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(), nWSR );

    }
    example.getPrimalSolution(data_optimal_solution_.data());
//

          if(ret == qpOASES::SUCCESSFUL_RETURN){
        // Get the solution
//               qpPlane_->getPrimalSolution(data_optimal_solution_.data());

                return true;
          }else{

            std::cout <<"Plane optimization failed :\n " <<  std::endl;
                         std::cout <<"return value is :  " <<  ret  << '\n';
                         std::cout <<"qpProblem status is : " << example.getStatus()<<'\n';
                         std::cout <<"qpProblem obj is : " << example.getObjVal()<<'\n';
                         std::cout <<"qpProblem cst number is : " << example.getNC()<<'\n';
                         std::cout <<"qpProblem variable number is : " << example.getNV()<<'\n';
                         std::cout <<"qpProblem is initialized ? " << example.isInitialised()<<'\n';
                         std::cout <<"qpProblem is solved ? " << example.isSolved()<<'\n';
                         std::cout <<"qpProblem is infeasible ? " << example.isInfeasible()<<'\n';
                         std::cout <<"qpProblem is Unbounded ? " << example.isUnbounded()<<'\n';
                         std::cout <<" print level : " << example.getPrintLevel() << std::endl;

              return false;
          }

}
