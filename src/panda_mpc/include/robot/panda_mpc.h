//
// Created by zheng on 12/05/2021.
//

#ifndef CATKIN_WS_PANDA_CONTROLLER_H
#define CATKIN_WS_PANDA_CONTROLLER_H

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>



#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <franka/robot.h>
#include <franka/robot_state.h>

#include <controller/controller.hpp>
namespace panda_mpc{


class PandaMPCController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::PositionJointInterface,
                                                hardware_interface::VelocityJointInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {

public:
    /**
    * @brief Franka Panda initialization routine
    */
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;

    /**
    * @brief Franka Panda starting routine
    */
    void starting(const ros::Time&) override;

    /**
    * @brief Franka Panda controller update routine
    */
    void update(const ros::Time&, const ros::Duration& period) override;


private:

    Controller::Controller qp;

    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

    hardware_interface::VelocityJointInterface* velocity_joint_interface_;
    hardware_interface::PositionJointInterface* position_joint_interface_;
    std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
    std::vector<hardware_interface::JointHandle> position_joint_handles_;
    std::vector<hardware_interface::JointHandle> joint_handles_;
    std::string control_level;
};



}


#endif //CATKIN_WS_PANDA_CONTROLLER_H
