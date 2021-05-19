#include <robot/robot_model.h>


namespace robot{


bool robotModel::Init(ros::NodeHandle &node_handle){


  // get robot description
  double timeout;
  node_handle.param("timeout", timeout, 0.005);
  std::string urdf_param;
  node_handle.param("urdf_param", urdf_param, std::string("/robot_description"));
  double eps = 1e-5;
  // Initialize KDL chain
  ik_solver_.reset(new TRAC_IK::TRAC_IK(root_link_, tip_link_, urdf_param, timeout, eps));
  bool valid = ik_solver_->getKDLChain(chain_);

  if (!valid) {
      ROS_ERROR_STREAM("There was no valid KDL chain found");
      return false;
  }
  // Get the limits from the urdf
  valid = ik_solver_->getKDLLimits(ll_,ul_);


  if (!valid) {
      ROS_ERROR_STREAM("There were no valid KDL joint limits found");
      return false;
  }


  assert(chain_.getNrOfJoints() == ll_.data.size());
  assert(chain_.getNrOfJoints() == ul_.data.size());

  if(chain_.getNrOfSegments() == 0)
      ROS_WARN("KDL chain empty !");

  ROS_INFO_STREAM("  Chain has "<<chain_.getNrOfJoints()<<" joints");
  ROS_INFO_STREAM("  Chain has "<<chain_.getNrOfSegments()<<" segments");
  for(unsigned int i=0; i<chain_.getNrOfSegments(); ++i)
      ROS_INFO_STREAM("    "<<chain_.getSegment(i).getName());


  // Initialize the various solvers
  fksolver_.reset(new KDL::ChainFkSolverPos_recursive(chain_)); // Forward kinematic solver for geometric purpose
  fkvelsolver_.reset(new KDL::ChainFkSolverVel_recursive(chain_));// Forward kinematic solver for kinematic purpose
  chainjacsolver_.reset(new KDL::ChainJntToJacSolver(chain_));  // To get the jacobian



  // INITIALIZE VARIABLES
  J_.resize(chain_.getNrOfJoints());
  M_.resize(chain_.getNrOfJoints());

  return true;
}

}



