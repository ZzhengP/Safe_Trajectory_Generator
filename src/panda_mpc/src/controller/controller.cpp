#include <controller/controller.hpp>


using namespace KDL;
using namespace std;

namespace Controller{
bool Controller::Init(ros::NodeHandle& node_handle,const Eigen::VectorXd& q_init, const Eigen::VectorXd& qd_init)
{
    ROS_WARN_STREAM ("Controller.cpp ");
    ROS_WARN("Using panda MPC controller !!! ");

    //--------------------------------------
    // INITIALIZE PUBLISHERS
    //--------------------------------------
    init_publishers(node_handle);

    //--------------------------------------
    // LOAD PARAMETERS
    //--------------------------------------
    load_parameters();

    //--------------------------------------
    // LOAD ROBOT
    //--------------------------------------
    if (!load_robot(node_handle, q_init, qd_init))
        return false;

    //--------------------------------------
    // INITIALIZE VARIABLES
    //--------------------------------------

    H_.resize(number_of_variables,number_of_variables);
    g_.resize(number_of_variables);
    lb_.resize(number_of_variables);
    ub_.resize(number_of_variables);
    A_.resize(number_of_constraints_,number_of_variables);
    lbA_.resize(number_of_constraints_);
    ubA_.resize(number_of_constraints_);
    joint_velocity_out_.resize(dof);

    //--------------------------------------
    // QPOASES
    //--------------------------------------
    qpoases_solver_.reset(new qpOASES::SQProblem(number_of_variables,number_of_constraints_,qpOASES::HST_POSDEF));

    // QPOases options
    qpOASES::Options options;
    // This options enables regularisation (required) and disable
    // some checks to be very fast !
    // options.setToDefault();
    options.setToMPC(); // setToReliable() // setToDefault()
    options.enableRegularisation = qpOASES::BT_FALSE; // since we specify the type of Hessian matrix, we do not need automatic regularisation
    options.enableEqualities = qpOASES::BT_TRUE; // Specifies whether equalities shall be  always treated as active constraints.
    qpoases_solver_->setOptions( options );
    qpoases_solver_->setPrintLevel(qpOASES::PL_NONE); // PL_HIGH for full output, PL_NONE for... none

    //--------------------------------------
    // INITIALIZE RBOT STATE
    //--------------------------------------
    q_in.q.data = q_init;
    q_in.qdot.data = qd_init;
    robot_model_->JntToCart(q_in.q, X_curr_);
    //--------------------------------------
    // BUILD TRAJECTORY
    //--------------------------------------
    std::string panda_traj_path = ros::package::getPath("panda_traj");
    std::string trajectory_file = panda_traj_path+"/trajectories/go_to_point.csv";
    std::string csv_file_name = trajectory_file;
    trajectory.Load(csv_file_name);
    trajectory.Build(X_curr_, true);
    traj_properties_.play_traj_ = true;

    next_tf_.M = X_curr_.M;
    next_tf_.p[0] = 0.4;
    next_tf_.p[1] = 0.4;
    next_tf_.p[2] = 0.25;
    next_point_.x = 0.4;
    next_point_.y = 0.4;
    next_point_.z = 0.25;
    preview_point_ = next_point_;
    x_err.setConstant(1);
    init_pos_attend_ = false;
    sub_goal_attend_ = false;
    wait = false;
    execute =true;
    ROS_WARN_STREAM(" Trajectory computed ");

    // ----------------------- Init MPC trajectory attributs -------------------
    if(!InitMPCTraj(node_handle,q_init,qd_init))
      return false;

    // -------------------------------------------------------------------------




    return true;
}

Eigen::VectorXd Controller::Update(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const ros::Duration& period)
{

    double  time_dt = period.toSec();

    //Get robot current state
    q_in.q.data = q;
    q_in.qdot.data = qd;

    // Update the model
    robot_model_->JntToJac(q_in.q);
    robot_model_->JntToCart(q_in.q, X_curr_);


    // -------------------- Update First MPC trajectory ----------------------------

    Eigen::Vector3d error_task_A, error_task_B;

    error_task_A << X_curr_.p[0] - Goal_A_frame_.p[0], X_curr_.p[1] - Goal_A_frame_.p[1], X_curr_.p[2] - Goal_A_frame_.p[2];
    error_task_B << X_curr_.p[0] - Goal_B_frame_.p[0], X_curr_.p[1] - Goal_B_frame_.p[1], X_curr_.p[2] - Goal_B_frame_.p[2];

    if(init_pos_attend_ & execute & !wait){

      if(error_task_A.norm()<0.05){
        q_des_ = q_goal_B_;
        wait = true;
        wait_begin_ = ros::Time::now();
      }

      if(error_task_B.norm()<0.05){
        q_des_ = q_goal_A_;
        wait = true;
        wait_begin_ = ros::Time::now();
      }



      UpdateMPCTraj();
      begin_time_ = ros::Time::now();
      execute = false;
    }


    // Control the frequency of planning, MPC trajectory generation is called each delta t time;
    end_time_ = ros::Time::now();
    wait_end_ = ros::Time::now();
    ros::Duration duration = end_time_ - begin_time_;
    if(duration.toSec() > dt_) // Change This parameter influences robot's behaviors;
      execute = true;

    ros::Duration duration_wait = end_time_ - begin_time_;
    if(duration_wait.toSec() > 1) // Change This parameter influences robot's behaviors;
      wait = false;

    // ------------------------------------------------------------------------------

    preview_point_ = next_point_;
    X_traj_ = next_tf_;
    Xd_traj_.vel[0] = next_vel_.linear.x, Xd_traj_.vel[1] = next_vel_.linear.y, Xd_traj_.vel[2] = next_vel_.linear.z;
    Xd_traj_.rot[0] = next_vel_.angular.x, Xd_traj_.rot[1] = next_vel_.angular.y, Xd_traj_.rot[2] = next_vel_.angular.z ;
//     Proportionnal controller
    X_err_ = diff( X_curr_ , X_traj_ );
    tf::twistKDLToEigen(X_err_,x_err);
    tf::twistKDLToEigen(Xd_traj_,xd_traj_);
    xd_des_ = p_gains_.cwiseProduct(x_err) + xd_traj_;


    // Formulate QP problem such that
    // joint_velocity_out_ = argmin 1/2 qd^T H_ qd + qd^T g_
    //                         s.t     lbA_ < A_ qd << ubA_
    //                                     lb_ < qd < ub_

    J = robot_model_->getJacobian().data;
    M = robot_model_->getJntInertial().data;

    H_ =  2.0 * regularisation_weight_ * Eigen::MatrixXd::Identity(7,7);
    g_ = -2.0 * regularisation_weight_ * p_gains_qd_.cwiseProduct((q_mean_ - q));

    H_ +=  2.0 *  J.transpose() * J;
    g_ += -2.0 *  J.transpose() * xd_des_;

    H_ += regularisation_weight_ * Eigen::MatrixXd::Identity(7,7);
    g_ += - regularisation_weight_ * joint_velocity_out_;
    double horizon_dt = 15 * time_dt;

    ub_ = qd_max_;
    lb_ = qd_min_;

    A_.block(0,0,7,7) = horizon_dt * Eigen::MatrixXd::Identity(7,7);
    ubA_.segment(0,7) = robot_model_->getJntul().data - q;
    lbA_.segment(0,7) = robot_model_->getJntll().data - q;

    // number of allowed compute steps
    int nWSR = 1e6;

    // Let's compute !
    qpOASES::returnValue ret;
    static bool qpoases_initialized = false;

    if(!qpoases_initialized){
        // Initialise the problem, once it has found a solution, we can hotstart
        ret = qpoases_solver_->init(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR);

        // Keep init if it didn't work
        if(ret == qpOASES::SUCCESSFUL_RETURN)
        qpoases_initialized = true;
    }
    else{
        // Otherwise let's reuse the previous solution to find a solution faster
        ret = qpoases_solver_->hotstart(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR);

        if(ret != qpOASES::SUCCESSFUL_RETURN)
        qpoases_initialized = false;
    }

    // Zero velocity if no solution found
    joint_velocity_out_.setZero();

    // If successful_return get the primal solution
    if(ret == qpOASES::SUCCESSFUL_RETURN)
        qpoases_solver_->getPrimalSolution(joint_velocity_out_.data());
    else
        ROS_WARN_STREAM("QPOases failed! Sending zero velocity");

    // Publish some messages
    do_publishing();

    return joint_velocity_out_;
}



bool Controller::UpdateMPCTraj(){




  for (size_t i(0); i < N_; i++){
    q_des_mpc_.segment(i*dof, dof) = q_des_.data;
  }

  solution_precedent_ = solution_;

  state_.head(dof) =  q_in.q.data;
  state_.tail(dof) = q_in.qdot.data;

  trajectory_generation->getRobotModel()->setJntState(state_.head(dof), state_.tail(dof));

  q_horizon_ = trajectory_generation->getJointHorizon();
  qd_horizon_ = trajectory_generation->getJointvelHorizon();

  solution_ = trajectory_generation->update(state_,q_horizon_,qd_horizon_,q_des_mpc_,qd_des_mpc_,solution_precedent_,J);

  state_ = state_A_*state_ + state_B_*solution_.head(dof);




  q_.data = state_.head(dof);
  q_mpc_.q.data = state_.head(dof);
  q_mpc_.qdot.data = state_.tail(dof);

  trajectory_generation->getRobotModel()->JntToCart(q_,X_mpc_);
  trajectory_generation->getRobotModel()->JntToJac(q_mpc_.q,kdl_J);
  ee_vel_ = kdl_J.data*q_mpc_.qdot.data;

  next_point_.x = X_mpc_.p[0];
  next_point_.y = X_mpc_.p[1];
  next_point_.z = X_mpc_.p[2];

  next_vel_.linear.x = ee_vel_[0];
  next_vel_.linear.y = ee_vel_[1];
  next_vel_.linear.z = ee_vel_[2];
  next_vel_.angular.x = 0;
  next_vel_.angular.y = 0;
  next_vel_.angular.z = 0;


  next_tf_.p[0] = next_point_.x;
  next_tf_.p[1] = next_point_.y;
  next_tf_.p[2] = next_point_.z;
  next_tf_.M = X_mpc_.M;

  return true;
}

bool Controller::InitMPCTraj(ros::NodeHandle &node_handle, const Eigen::VectorXd &q_init, const Eigen::VectorXd &qd_init)
{


  //--------------------------------------
  // Initialize MPC trajecotry generator
  //--------------------------------------
  node_handle.getParam("/panda_mpc/dt_", dt_);
  node_handle.getParam("/panda_mpc/N_", N_);
  ros::param::get("/panda_mpc/init_pos_attend_", init_pos_attend_);

  kdl_J.resize(dof);
  q_mpc_.resize(dof);

  q_des_mpc_.resize(dof*N_), qd_des_mpc_.resize(dof*N_), qdd_des_mpc_.resize(dof*N_);

  for (size_t i(0); i<N_; i++){
    q_des_mpc_.segment(i*dof,dof) = q_init;
    qd_des_mpc_.segment(i*dof,dof) = qd_init;
  }

  qdd_des_mpc_.setZero();
  q_horizon_.resize(dof*N_);
  qd_horizon_.resize(dof*N_);
  q_horizon_ = q_des_mpc_;
  qd_horizon_ = qd_des_mpc_;

  state_.resize(2*dof);
  state_.segment(0,dof) = q_init, state_.segment(dof,dof) = qd_init;

  solution_.resize(N_*dof);
  solution_precedent_.resize(N_*dof);
  solution_.setZero();
  solution_precedent_.setZero();

  state_A_.resize(2*dof, 2*dof);
  state_B_.resize(2*dof,dof);

  Goal_A_frame_.p[0] = 0.4, Goal_A_frame_.p[1] = 0.4, Goal_A_frame_.p[2] = 0.2;
  Goal_A_frame_.M = X_curr_.M;

  Goal_B_frame_.p[0] = 0.4, Goal_B_frame_.p[1] = -0.4, Goal_B_frame_.p[2] = 0.2;
  Goal_B_frame_.M = X_curr_.M;

  q_des_.resize(dof);
  q_goal_A_.resize(dof);
  q_goal_B_.resize(dof);
  ee_vel_.resize(6);

  trajectory_generation.reset(new planning::trajGen(node_handle,q_init,qd_init));
  trajectory_generation->getRobotModel()->CartToJnt(Goal_A_frame_,q_des_);
  trajectory_generation->getRobotModel()->CartToJnt(Goal_A_frame_,q_goal_A_);
  trajectory_generation->getRobotModel()->CartToJnt(Goal_B_frame_,q_goal_B_);

  if(!trajectory_generation->init(node_handle,next_tf_)){
    ROS_ERROR_STREAM("Unable to initialize properly parameters: exit");
    return false;
  }

  for (size_t i(0); i < N_; i++){
    q_des_mpc_.segment(i*dof, dof) = q_des_.data;
  }

  state_A_ = trajectory_generation->getStateA();
  state_B_ = trajectory_generation->getStateB();


  return true;
}



void Controller::BuildTrajectory(KDL::Frame X_curr_){
    trajectory.Build(X_curr_, false);
    publishTrajectory();
}

void Controller::publishTrajectory()
{
    panda_traj::PublishTraj publish_traj_;
    publish_traj_ = trajectory.publishTrajectory();

    nav_msgs::Path path_ros;
    path_ros.poses = publish_traj_.path_ros_.poses;
    path_ros.header.frame_id = root_link_;
    path_ros.header.stamp = ros::Time::now();
    geometry_msgs::PoseArray pose_array;
    pose_array.poses = publish_traj_.pose_array_.poses;
    pose_array.header.frame_id = root_link_;
    pose_array.header.stamp = ros::Time::now();

    if (pose_array_publisher_.trylock())
    {
        pose_array_publisher_.msg_.header.stamp = ros::Time::now();
        pose_array_publisher_.msg_.header.frame_id = root_link_;
        pose_array_publisher_.msg_ = pose_array;
        pose_array_publisher_.unlockAndPublish();
    }
    if (path_publisher_.trylock())
    {
        path_publisher_.msg_.header.stamp = ros::Time::now();
        path_publisher_.msg_.header.frame_id = root_link_;
        path_publisher_.msg_ = path_ros;
        path_publisher_.unlockAndPublish();
    }
    ROS_INFO_STREAM(" Trajectory published ");
}


void Controller::init_publishers(ros::NodeHandle& node_handle){
    //Realtime safe publishers
    pose_array_publisher_.init(node_handle, "Pose_array", 1);
    path_publisher_.init(node_handle, "Ros_Path", 1);
    panda_rundata_publisher.init(node_handle, "panda_rundata", 1);
    pose_curr_publisher_.init(node_handle,"ee_pose", 1);

}

void Controller::load_parameters(){
    ROS_INFO_STREAM ( "------------- Loading parameters -------------" );
    qd_min_.resize(7);
    getRosParam("/panda_mpc/qd_min_",qd_min_);
    qd_max_.resize(7);
    getRosParam("/panda_mpc/qd_max_",qd_max_);
    p_gains_.resize(6);
    getRosParam("/panda_mpc/p_gains_",p_gains_);
    d_gains_.resize(6);
    getRosParam("/panda_mpc/d_gains_",d_gains_);
    p_gains_qd_.resize(7);
    getRosParam("/panda_mpc/p_gains_qd_",p_gains_qd_);
    q_mean_.resize(7);
    getRosParam("/panda_mpc/q_mean_",q_mean_);
    getRosParam("/panda_mpc/regularisation_weight_",regularisation_weight_);
    getRosParam("/panda_mpc/root_link_",root_link_);
    getRosParam("/panda_mpc/tip_link_",tip_link_);



    ROS_INFO_STREAM ( "------------- Parameters Loaded -------------" );
}

bool Controller::load_robot(ros::NodeHandle& node_handle, const Eigen::VectorXd& q_init, const Eigen::VectorXd qd_init)
{
   ROS_INFO_STREAM ( "------------- Loading robot -------------" );

    updateUI_service = node_handle.advertiseService("updateUI", &Controller::updateUI, this);
//    updateTraj_service = node_handle.advertiseService("updateTrajectory", &Controller::updateTrajectory, this); // Lucas 's code
//    updateNextTraj_service_ = node_handle.advertiseService("/panda_mpc/next_point", &Controller::updateTrajectoryPoint,this);
//    trajectory_msg_subscriber_ = node_handle.subscribe("/trajectory_generation/next_point", 1000, &Controller::updateTrajectoryPoint,this);
    // Initialize robot model
    robot_model_.reset(new robot::RobotModel(node_handle, root_link_, tip_link_));
    robot_model_->Init(node_handle, q_init, qd_init);
    dof = robot_model_->getNrOfJoints();
    number_of_variables = dof;
    number_of_constraints_ = dof;

    ROS_INFO_STREAM ( "Number of variables : " << number_of_variables );
    ROS_INFO_STREAM ( "Number of constraints : " << number_of_constraints_);

    ROS_INFO_STREAM ( "------------- robot Loaded -------------" );

    return true;
}




void Controller::do_publishing()
{
    // Publishing
    tf::poseKDLToMsg(X_curr_, X_curr_msg_);
    tf::poseKDLToMsg(X_traj_, X_traj_msg_);
    tf::twistKDLToMsg(X_err_, X_err_msg_);

    // Publish custom message define in the msg folder
    if (panda_rundata_publisher.trylock())
    {
        panda_rundata_publisher.msg_.header.stamp = ros::Time::now();
        panda_rundata_publisher.msg_.header.frame_id = root_link_;
        panda_rundata_publisher.msg_.X_err = X_err_msg_;
        panda_rundata_publisher.msg_.play_traj_ = traj_properties_.play_traj_;
        panda_rundata_publisher.msg_.tune_gains_ = traj_properties_.gain_tunning_ ;
        panda_rundata_publisher.unlockAndPublish();
    }

    //Publish robot current pose
    if (pose_curr_publisher_.trylock())
    {
        tf::poseKDLToMsg(X_curr_, X_curr_msg_);
        geometry_msgs::PoseStamped X_curr_stamp;
        pose_curr_publisher_.msg_.header.stamp = ros::Time::now();
        pose_curr_publisher_.msg_.header.frame_id = root_link_;
        pose_curr_publisher_.msg_.pose = X_curr_msg_;
        pose_curr_publisher_.unlockAndPublish();
    }

    //Publish robot desired pose
    if (pose_des_publisher_.trylock())
    {
        tf::poseKDLToMsg(X_traj_, X_traj_msg_);
        geometry_msgs::PoseStamped X_des_stamp;
        pose_des_publisher_.msg_.header.stamp = ros::Time::now();
        pose_des_publisher_.msg_.header.frame_id = root_link_;
        pose_des_publisher_.msg_.pose = X_traj_msg_;
        pose_des_publisher_.unlockAndPublish();
    }
}

// Ros service to interact with the code
bool Controller::updateUI(panda_mpc::UI::Request &req, panda_mpc::UI::Response &resp)
{
    traj_properties_.play_traj_ = req.play_traj;
//    req.publish_traj = true;

    if (req.publish_traj)
        publishTrajectory();
    if (req.build_traj)
        BuildTrajectory(X_curr_);

    if (req.exit_)
    {
        ros::shutdown();
        exit(0);
    }
    resp.result = true;

    return true;
}

// Ros service to update the trajectory
bool Controller::updateTrajectory(panda_traj::UpdateTrajectory::Request& req, panda_traj::UpdateTrajectory::Response& resp){

  trajectory.Load(req.csv_traj_path);
  trajectory.Build(X_curr_, req.verbose);
  publishTrajectory();
  std::cout << "Received waypoint computing traj and publishing" << std::endl;

  return true;
}

// Subscribe to a trajectory generation topic from the next point
void Controller::updateTrajectoryPoint(const panda_mpc::trajectoryMsg::ConstPtr& traj_msg)
{


  if(traj_msg != nullptr){

    next_point_ = traj_msg->next_point;
    next_vel_ = traj_msg->next_vel;

    next_tf_.p[0] = next_point_.x;
    next_tf_.p[1] = next_point_.y;
    next_tf_.p[2] = next_point_.z;
   }else{
    next_tf_.p[0] = preview_point_.x;
    next_tf_.p[1] = preview_point_.y;
    next_tf_.p[2] = preview_point_.z;
  }

//  auto error = KDL::diff(next_tf_.p, X_curr_.p);
//  if(error.Norm() > 0.005){
//    trajectory.Load(next_tf_, 0.2);
//    trajectory.Build(X_curr_, true);
//  }
//  publishTrajectory();
  std::cout << "Received next point computing traj and publishing" << std::endl;

}


// End of namespace
}
