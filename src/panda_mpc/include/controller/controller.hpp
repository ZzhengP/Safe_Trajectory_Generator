#ifndef CATKIN_WS_CONTROLLER_H
#define CATKIN_WS_CONTROLLER_H

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <controller_interface/multi_interface_controller.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <ros/package.h>
#include <Eigen/Dense>

#include <qpOASES.hpp>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <realtime_tools/realtime_publisher.h>


#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>



namespace Controller {

class Controller{

public:

  /**
   * @brief bool Init
   * @param nh
   * @param q_init
   * @param qd_init
   * @return
   */
  bool Init(ros::NodeHandle& nh, const Eigen::VectorXd &q_init , const Eigen::VectorXd &qd_init );


};

}


#endif
