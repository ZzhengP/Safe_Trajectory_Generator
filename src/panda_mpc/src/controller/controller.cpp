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
    joint_velocity_out_precedent_.resize(dof);
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


    x_err.setConstant(1);
    init_pos_attend_ = false;
    sub_goal_attend_ = false;
    wait = false;
    execute =true;
    traj_index_ = 0;
    ROS_WARN_STREAM(" Trajectory computed ");

    // ----------------------- Init MPC trajectory attributs -------------------
    if(!InitMPCTraj(node_handle,q_init,qd_init))
      return false;

    // -------------------------------------------------------------------------


    joints_coefficient_matrix_.resize(dof);

    for (int i(0); i<dof; i++){
      joints_coefficient_matrix_[i].resize(N_,4);
      joints_coefficient_matrix_[i].setZero();
    }

    joints_array.resize(N_+1);
    for (int i(0); i<N_+1;i++){
      joints_array[i].resize(dof);
      joints_array[i].q.data.setZero();
      joints_array[i].qdot.data.setZero();
      joints_array[i].qdotdot.data.setZero();

    }
    return true;
}

Eigen::VectorXd Controller::Update(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const ros::Duration& period)
{

    double  time_dt = period.toSec();
    //time_dt = 0.05;
    //Get robot current state
    q_in.q.data = q;
    q_in.qdot.data = qd;

    // Update the model
    robot_model_->JntToJac(q_in.q);
    robot_model_->JntToCart(q_in.q, X_curr_);


    // -------------------- Update First MPC trajectory ----------------------------

    Eigen::Vector3d error_task_A, error_task_B;

    // ------------------------------------------------------------------------------
    traj_index_++;
    if(traj_index_ > 50){
      traj_index_ = 50;
    }


      q_mpc_.qdot.data = q_.qdot.data + q_mpc_.qdotdot.data  * 0.05;
      q_mpc_.q.data = q_.q.data + q_.qdot.data * time_dt + 0.5*q_mpc_.qdotdot.data*pow(0.05,2);

//    Proportionnal controller

     q_mpc_.q.data[6] = 0;
     q_mpc_.qdot.data[6] = 0;

     jnt_err =  q_mpc_.q.data - q_in.q.data ;
     jnt_des_ = jnt_err + q_mpc_.qdot.data;
//     jnt_des_ = jnt_err;
    // Formulate QP problem such that
    // joint_velocity_out_ = argmin 1/2 qd^T H_ qd + qd^T g_
    //                         s.t     lbA_ < A_ qd << ubA_
    //                                     lb_ < qd < ub_

    J = robot_model_->getJacobian().data;
    M = robot_model_->getJntInertial().data;

    H_ =  2.0 * regularisation_weight_ * Eigen::MatrixXd::Identity(7,7);
    g_ = -2.0 * regularisation_weight_ * p_gains_qd_.cwiseProduct((q_mean_ - q));

//    H_ +=   1* regularisation_weight_ * Eigen::MatrixXd::Identity(7,7);
//    g_ += - 1 * regularisation_weight_ * joint_velocity_out_precedent_;

//    H_ +=   10* regularisation_weight_ * Eigen::MatrixXd::Identity(7,7)*time_dt;
//    g_ += - 10 * regularisation_weight_ * q_in.q.data;


    H_ += Eigen::MatrixXd::Identity(7,7);
    g_ += - jnt_des_;

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

    joint_velocity_out_precedent_ = joint_velocity_out_;

    return joint_velocity_out_;
}




bool Controller::InitMPCTraj(ros::NodeHandle &node_handle, const Eigen::VectorXd &q_init, const Eigen::VectorXd &qd_init)
{


  //--------------------------------------
  // Initialize MPC trajecotry generator
  //--------------------------------------
  node_handle.getParam("/panda_mpc/dt_", dt_);
  node_handle.getParam("/panda_mpc/N_", N_);
  ros::param::get("/panda_mpc/init_pos_attend_", init_pos_attend_);

  q_mpc_.resize(dof);


  q_mpc_.q.data = q_init;
  q_mpc_.qdot.data = qd_init;
  q_mpc_.qdot.data.setZero();
  q_mpc_.qdotdot.data.setZero();
  q_horizon_.resize(dof*N_);
  qd_horizon_.resize(dof*N_);

  solution_.resize(N_*dof);
  solution_precedent_.resize(N_*dof);
  solution_.setZero();
  solution_precedent_.setZero();


  Goal_A_frame_.p[0] = 0.5, Goal_A_frame_.p[1] = 0.5, Goal_A_frame_.p[2] = 0.2;
  Goal_A_frame_.M = X_curr_.M;

  Goal_B_frame_.p[0] = 0.5, Goal_B_frame_.p[1] = -0.5, Goal_B_frame_.p[2] = 0.2;
  Goal_B_frame_.M = X_curr_.M;

  q_des_.resize(dof);
  q_goal_A_.resize(dof);
  q_goal_B_.resize(dof);
  ee_vel_.resize(6);


  trajectory_generation.reset(new planning::trajGen(node_handle,q_init,qd_init));
  trajectory_generation->getRobotModel()->CartToJnt(Goal_A_frame_,q_des_);
  trajectory_generation->getRobotModel()->CartToJnt(Goal_A_frame_,q_goal_A_);
  trajectory_generation->getRobotModel()->CartToJnt(Goal_B_frame_,q_goal_B_);

  q_mpc_.q.data = q_goal_A_.data;

  q_.resize(dof);
  q_.q = q_goal_A_;
  q_.qdot.data.setZero();
  ROS_WARN_STREAM("q_goal A : " << q_goal_A_.data);
  ROS_WARN_STREAM("q_goal B : " << q_goal_B_.data);


  jnt_err.resize(dof);

  delete trajectory_generation.get();
  return true;
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

    trajectory_msg_subscriber_ = node_handle.subscribe("/mpc_solution", 1, &Controller::updateTrajectoryPoint,this);
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




// Subscribe to a trajectory generation topic from the next point
void Controller::updateTrajectoryPoint(const panda_mpc::trajectoryAcceleration::ConstPtr& traj_acc)
{

//  ROS_WARN_STREAM("receive computed MPC solution");

//  joints_array[0].qdotdot.data = joints_array[1].qdotdot.data ;
 if(traj_acc != nullptr){



    for (int i(0); i<dof ; i++){
      q_mpc_.qdotdot.data[i] = traj_acc->jntAcc.at(i);
      q_.q.data[i] = traj_acc->jntAcc.at(i+dof);
      q_.qdot.data[i] = traj_acc->jntAcc.at(i+2*dof);

    }
     traj_index_ = 0;
   //  q_ = q_in;

  }


}


// End of namespace
}
