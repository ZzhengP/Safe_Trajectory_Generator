// Copyright (c) 2017 Franka Emika GmbH0
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <cmath>
#include <memory>
#include <chrono>
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <robot/panda_mpc.h>

using namespace std;

namespace panda_mpc{


bool PandaMPCController::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) {

    //--------------------------------------
    // LOAD ROBOT
    //--------------------------------------
    string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
        ROS_ERROR_STREAM("CartesianImpedanceExampleController: Could not read parameter arm_id");
        return false;
    }
    if (!node_handle.getParam("control_level", control_level)) {
        ROS_ERROR_STREAM("Could not read parameter control_level");
        return false;
    }

    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names)) {
        ROS_ERROR("JointVelocityExampleController: Could not parse joint names");
    }
    if (joint_names.size() != 7) {
        ROS_ERROR_STREAM("JointVelocityExampleController: Wrong number of joint names, got "
        << joint_names.size() << " instead of 7 names!");
        return false;
    }

    auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR("JointVelocityExampleController: Could not get state interface from hardware");
        return false;
    }
    try {
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>( state_interface->getHandle("panda_robot"));
    } catch (const hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM(
            "JointVelocityExampleController: Exception getting state handle: " << e.what());
            return false;
    }

    if (control_level == "velocity")
    {
        velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
        if (velocity_joint_interface_ == nullptr) {
            ROS_ERROR(
                "JointVelocityExampleController: Error getting velocity joint interface from hardware!");
                return false;
        }
         velocity_joint_handles_.resize(7);
        for (size_t i = 0; i < 7; ++i) {
            try {
                velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
            } catch (const hardware_interface::HardwareInterfaceException& ex) {
                ROS_ERROR_STREAM(
                "JointVelocityExampleController: Exception getting joint handles: " << ex.what());
                return false;
            }
        }
    }
    else if (control_level == "position"){
      position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
      if (position_joint_interface_ == nullptr) {
          ROS_ERROR(
              "JointpositionExampleController: Error getting position joint interface from hardware!");
              return false;
      }
       position_joint_handles_.resize(7);
      for (size_t i = 0; i < 7; ++i) {
          try {
              position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
          } catch (const hardware_interface::HardwareInterfaceException& ex) {
              ROS_ERROR_STREAM(
              "JointPositionExampleController: Exception getting joint handles: " << ex.what());
              return false;
          }
      }
    }
    else
    {
        ROS_ERROR_STREAM("control_level must be either velocitiy or torque");
        return false;
    }

    Eigen::VectorXd q_init, qd_init;

    q_init.resize(7);
    qd_init.resize(7);
    initial_joint_pos_.resize(7);

    franka::RobotState robot_state = state_handle_->getRobotState();
    //Get robot current state
    for (int i =0 ; i<7 ; i++)
    {
        q_init(i) = robot_state.q[i];
        qd_init(i) = robot_state.dq[i];
    }

    if (control_level == "velocity")
    {
        for (size_t i = 0; i < 7; ++i)
        {
            initial_joint_pos_(i) = velocity_joint_handles_[i].getPosition();
        }
    }
    else if (control_level == "position"){
      for (size_t i = 0; i < 7; ++i)
      {

           initial_joint_pos_(i) = position_joint_handles_[i].getPosition();
       }
    }

    qp.Init(node_handle, q_init, qd_init);

    joint_command_precedent_.resize(7);
    joint_command_precedent_.setZero();
    joint_command_dx_preview_.resize(7);
    joint_command_dx_preview_.setZero();
    joint_command_dx.resize(7);
    joint_command_dx.setZero();
    return true;
}

void PandaMPCController::starting(const ros::Time&)
{
    ROS_WARN_STREAM("Starting QP Controller on the real Panda");
    elapsed_time_ = ros::Duration(0.);
}


void PandaMPCController::update(const ros::Time&, const ros::Duration& period) {

    //--------------------------------------
    // ROBOT STATE
    //--------------------------------------



    // get state variables
    franka::RobotState robot_state = state_handle_->getRobotState();

    Eigen::VectorXd q_,qd_;
    q_.resize(7);
    qd_.resize(7);
    //Get robot current state
    for (int i =0 ; i<7 ; i++)
    {
        q_(i) = robot_state.q[i];
        qd_(i) = robot_state.dq[i];
    }

    Eigen::VectorXd joint_command_;
    joint_command_.resize(7);
    Eigen::VectorXd gravity_comp;
    gravity_comp.resize(7);
    joint_command_ = qp.Update(q_,qd_,period);





    if (control_level == "velocity")
    {
        for (size_t i = 0; i < 7; ++i)
        {

          double alpha = 0.9;

          double x_hat = alpha*joint_command_(i) + (1-alpha)*joint_command_precedent_(i);
          joint_command_precedent_(i) = x_hat;
           velocity_joint_handles_[i].setCommand(x_hat);

        }
    }
    else if (control_level == "position"){
      for (size_t i = 0; i < 7; ++i)
      {
           position_joint_handles_[i].setCommand(position_joint_handles_[i].getPosition() + 0.01*(joint_command_(i) -position_joint_handles_[i].getPosition()) );

       }
    }

}



}  // namespace panda_mpc

PLUGINLIB_EXPORT_CLASS(panda_mpc::PandaMPCController,
controller_interface::ControllerBase)

