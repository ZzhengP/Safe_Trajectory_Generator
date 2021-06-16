#include <planning/traj_generation.hpp>

namespace planning {

  bool trajGen::init(ros::NodeHandle& node_handle, KDL::Frame init_frame){


    // ------------------------- Task -------------------------
    if(!mpc_task_->init(q_init_)){
      ROS_ERROR_STREAM("Failed to initialize MPC task parameters ");
      return  false;
     }else {
      ROS_WARN_STREAM("Success to initialize MPC task parameters ");
      mpc_task_->print();

     }

    ros::param::get("/panda_mpc/dsafe_",dsafe_);

    // ------------------------- Constraint -------------------------
    if(!mpc_constraint_->init(q_min_mpc_,q_max_mpc_,qd_min_mpc_,qd_max_mpc_,dsafe_)){
      ROS_ERROR_STREAM("Failed to initialize MPC constraints parameters ");
      return false;
    }else{
      ROS_WARN_STREAM("Success to initialize MPC constraints parameters ");
      mpc_constraint_->print();
    }

                            //-Add new constraint -
    optimization::constraint new_constraint;
    new_constraint.constraint_name_ = "obstacle_avoidance_constraint";
    new_constraint.lbA_.resize(2*(N_-1));
    new_constraint.lbA_.setConstant(-10000);
    new_constraint.ubA_.resize(2*(N_-1));
    new_constraint.ubA_.setConstant(10000);
    new_constraint.A_.resize(2*(N_-1), N_*dof_);
    new_constraint.constraint_size_ = 2*(N_-1);
    mpc_constraint_->addConstraint(new_constraint);
    // ------------------------- QP Solver -------------------------
    // qp_oases solver init
    nV_ = N_*dof_;
    nbrCst_ = mpc_constraint_->getConstraintNumber();

    qpoases_solver_.configureQP(nV_,nbrCst_);
    std::cout << "qp solver number of constraint : " << nbrCst_ << '\n';
    ros::param::get("/panda_mpc/robot_member_", robot_member_);
    ros::param::get("/panda_mpc/robot_vertices_", robot_vertices_);
    ros::param::get("/panda_mpc/obstacle_member_", obstacle_member_);
    ros::param::get("/panda_mpc/obstacle_vertices_", obstacle_vertices_);

    path_pub_ = node_handle.advertise<nav_msgs::Path>("/mpc_path",1);
    // ---------------- Initialize plane generation module ----------------------
    plane_table_pub_ = node_handle.advertise<visualization_msgs::Marker>("/table_plane", 1);
    plane_shape_pub_ = node_handle.advertise<visualization_msgs::Marker>("/separating_plane", 1);
    obstacle_shape_pub_ = node_handle.advertise<visualization_msgs::Marker>("/obstacle_shape",1);

    Eigen::Vector3d obstacle_center;

    obstacle_center << 0.5, -0.0, 0.15;


    obsVertices_.resize(3,obstacle_vertices_);
    obsVertices_ << obstacle_center[0], obstacle_center[1], obstacle_center[2];

    obsVerticesAugmented_.resize(obstacle_member_);
    for (int j(0); j<obstacle_member_; j++){
      obsVerticesAugmented_[j].resize(3,obstacle_vertices_*N_);
      for (int i(0); i<N_; i++){
           obsVerticesAugmented_[j].block(0,i*obstacle_vertices_,3,obstacle_vertices_) = obsVertices_;
      }
    }


    robotVertices_.resize(3, robot_vertices_);
    robotVertices_ << init_frame.p[0], init_frame.p[1], init_frame.p[2];
    robotVerticesAugmented_.resize(robot_member_);
    robotVerticesAugmented_[0].resize(3, robot_vertices_*N_);

    for (int i(0); i<N_; i++){
      robotVerticesAugmented_[0].block(0,i,3,1) = robotVertices_;
    }

    int nbrConstraint = 10;
    plane_generation.reset(new planning::plane(node_handle, N_, nbrConstraint, dsafe_));

    if (!plane_generation->init(robotVerticesAugmented_,obsVerticesAugmented_))
      ROS_WARN_STREAM("plane initialization failed");


    // add table surface avoidance
    table_plane_location_.resize(1);
    table_plane_location_[0].resize(5, N_-1);

    for (int i(0); i< N_-1 ; i++){

      table_plane_location_[0].block(0,i,5,1) << 0, 0, -1, 0., 0;

    }

    ROS_WARN_STREAM("Trajectory generation module initialized");





    return true;
  }



  Eigen::VectorXd trajGen::update(Eigen::VectorXd S, const Eigen::VectorXd &q_horizon, const Eigen::VectorXd & qd_horizon,
                                  const Eigen::VectorXd &q_des, const Eigen::VectorXd &qd_des,const Eigen::VectorXd &solution_precedent,
                                  Eigen::MatrixXd J){

    bool is_solved = false;
    mpc_param_ = robot_mpc_model_->getMPCParams();
    Eigen::VectorXd qdd_des(N_*dof_);
    qdd_des.setZero();

    q_horizon_precedent_ = robot_mpc_model_->getJntHorizon();
    // update robot model
    robot_mpc_model_->setJntState(S.segment(0,dof_), S.segment(dof_,dof_));
    robot_mpc_model_->update(S,solution_precedent);

    J_horizon_ = robot_mpc_model_->getMPCParams().J_horizon_;

    // Update robot desired task
    if(!mpc_task_->update(S, q_des,qd_des,qdd_des)){
      ROS_INFO_STREAM("failed to update task");
    }else{
//      ROS_INFO_STREAM("success to update task");

    }
    qpoases_solver_.H_ = mpc_task_->getHessien();
    qpoases_solver_.g_ = mpc_task_->getGradient();

    plane_location_ = plane_generation->GetPlane();


    mpc_constraint_->computeUpperBoundAndConstraint(S,
                                   robotVerticesAugmented_,
                                   plane_location_,
                                   J_horizon_,
                                   q_horizon,
                                   3);

//    mpc_constraint_->avoidanceTest(S,
//                                   robotVerticesAugmented_[0],
//                                   plane_location_[0].col(0),
//                                   J.block(0,0,3,7),
//                                   q_horizon.tail(2*dof_),
//                                    3);
    // Update constraint
    if(!mpc_constraint_->update(S, q_des.segment(0,dof_),
                                robotVerticesAugmented_,
                                table_plane_location_,
                                J_horizon_, q_horizon_precedent_)){
      ROS_INFO_STREAM("failed to update constraint");
    }
    else{
//          ROS_INFO_STREAM("success to update constraint");
    }
    qpoases_solver_.lb_.setConstant(-12);
    qpoases_solver_.ub_.setConstant(12);
    qpoases_solver_.lbA_ = mpc_constraint_->getLBA();
    qpoases_solver_.ubA_ = mpc_constraint_->getUBA();
    qpoases_solver_.A_ = mpc_constraint_->getConstraintA();

    is_solved = qpoases_solver_.solve();



    // ------------------- Update Plane -----------
    // update robot vertices horizon for plane generation

    robotVerticesAugmented_[0]= computeTipPositionHorizon(q_horizon,robot_vertices_);

    plane_generation->update(robotVerticesAugmented_,
                             obsVerticesAugmented_);

    publishABCDPlane(plane_generation->GetFirstPlane()(0,0),
                       plane_generation->GetFirstPlane()(1,0),
                       plane_generation->GetFirstPlane()(2,0),
                       -plane_generation->GetFirstPlane()(3,0));


    Eigen::Vector3d obstacle_center;
    obstacle_center << obsVertices_(0,0),obsVertices_(1,0),obsVertices_(2,0);

    publishObstacle(obstacle_center);
    publishPath();
    if (is_solved){
      return qpoases_solver_.optimal_solution_;
    }else{
      return qpoases_solver_.optimal_solution_.setZero();
    }
  }




  bool trajGen::publishABCDPlane(double A, double B, double C, double D,
                        double x_width, double y_width ){

      Eigen::Vector3d n(A,B,C);
      double distance = D/n.norm();
      Eigen::Vector3d center = -distance*n.normalized();

      Eigen::Isometry3d pose;
      pose.translation() = center;

      Eigen::Vector3d z_0 = Eigen::Vector3d::UnitZ();
      Eigen::Quaterniond q= Eigen::Quaterniond::FromTwoVectors(z_0,n);
      pose.linear() = q.toRotationMatrix();

      double height = 0.001;

      uint32_t shape = visualization_msgs::Marker::CUBE;
      visualization_msgs::Marker marker;
          // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = "/panda_link0";
      marker.header.stamp = ros::Time::now();
      marker.ns = "plane_shape";
      marker.id = 0;

          // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type = shape;

          // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      marker.action = visualization_msgs::Marker::ADD;

          // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
     marker.pose.position.x = center[0];
     marker.pose.position.y = center[1];
     marker.pose.position.z = center[2];
     marker.pose.orientation.x = q.x();
     marker.pose.orientation.y = q.y();
     marker.pose.orientation.z = q.z();
     marker.pose.orientation.w = q.w();

          // Set the scale of the marker -- 1x1x1 here means 1m on a side
     marker.scale.x = x_width;
     marker.scale.y = y_width;
     marker.scale.z = 0.01;

          // Set the color -- be sure to set alpha to something non-zero!
     marker.color.r = 0.0f;
     marker.color.g = 1.0f;
     marker.color.b = 0.0f;
     marker.color.a = 0.5;

     marker.lifetime = ros::Duration();
     plane_shape_pub_.publish(marker);

     visualization_msgs::Marker table;
         // Set the frame ID and timestamp.  See the TF tutorials for information on these.
     table.header.frame_id = "/panda_link0";
     table.header.stamp = ros::Time::now();
     table.ns = "table";
     table.id = 10;

         // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
     table.type = shape;

         // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
     table.action = visualization_msgs::Marker::ADD;

         // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    table.pose.position.x = 0;
    table.pose.position.y = 0;
    table.pose.position.z = 0.0;

         // Set the scale of the marker -- 1x1x1 here means 1m on a side
    table.scale.x = x_width;
    table.scale.y = y_width;
    table.scale.z = 0.01;

         // Set the color -- be sure to set alpha to something non-zero!
    table.color.r = 0.0f;
    table.color.g = 1.0f;
    table.color.b = 0.0f;
    table.color.a = 0.1;

    table.lifetime = ros::Duration();
    plane_table_pub_.publish(table);

      return true;
  }



  bool trajGen::publishObstacle(Eigen::Vector3d obstacle_center){

    uint32_t shape = visualization_msgs::Marker::SPHERE;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/panda_link0";
    marker.header.stamp = ros::Time::now();
    marker.ns = "obstacle_shape";
    marker.id = 1;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = obstacle_center[0];
    marker.pose.position.y = obstacle_center[1];
    marker.pose.position.z = obstacle_center[2];
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration();
    obstacle_shape_pub_.publish(marker);

    return true;
  }

  bool trajGen::publishPath(){

      nav_msgs::Path path;
      path.header.stamp = ros::Time::now();
      path.header.frame_id=  "/panda_link0";
      mpc_param_ = robot_mpc_model_->getMPCParams();

      Eigen::VectorXd x_horizon;
      x_horizon.resize(3*N_);
      x_horizon = mpc_param_.x_horizon_;

      for (int i(0); i < N_; i++){

          geometry_msgs::PoseStamped this_pose_stamped;
          this_pose_stamped.pose.position.x = x_horizon(3*i);
          this_pose_stamped.pose.position.y = x_horizon(3*i+1);
          this_pose_stamped.pose.position.z = x_horizon(3*i+2);
          this_pose_stamped.header.stamp=ros::Time::now();
          this_pose_stamped.header.frame_id= "/panda_link0";

          path.poses.push_back(this_pose_stamped);
      }


      path_pub_.publish(path);
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






