cmake_minimum_required(VERSION 3.0.2)
project(panda_mpc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
  qpOASES
  eigen_conversions
  panda_traj
  message_runtime
  trac_ik_lib
  ## Franka part
        controller_interface
        dynamic_reconfigure
        franka_hw
        franka_gripper
        geometry_msgs
        hardware_interface
        tf
        tf_conversions
        message_generation
        pluginlib
        realtime_tools
        rviz_visual_tools
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_message_files(
  FILES
  PandaRunMsg.msg
  trajectoryMsg.msg
  trajectoryAcceleration.msg
)

add_service_files(
  FILES
  UI.srv
  UpdateTrajectoryNextPoint.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs)

generate_dynamic_reconfigure_options(
        cfg/compliance_param.cfg
        cfg/desired_mass_param.cfg
        cfg/dual_arm_compliance_param.cfg
        cfg/teleop_param.cfg
        cfg/teleop_gripper_param.cfg
)


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES panda_mpc
        CATKIN_DEPENDS
            controller_interface
            dynamic_reconfigure
            eigen_conversions
            franka_hw
            franka_gripper
            geometry_msgs
            hardware_interface
            tf
            tf_conversions
            message_runtime
            pluginlib
            realtime_tools
            roscpp
            rviz_visual_tools
        DEPENDS Franka
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} )
#link_directories(${qpOASES_LIBRARY_DIRS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#include_directories(
# include
#  ${catkin_INCLUDE_DIRS}
#)

# build this project library

add_library(panda_mpc
            src/panda_simulation.cpp
            src/robot/panda_mpc.cpp
            src/robot/robot_model.cpp
            src/robot/robot_mpc_model.cpp
            src/optFormulation/task.cpp
            src/optFormulation/constraint.cpp
            src/optFormulation/qpsolver.cpp
            src/planning/traj_generation.cpp
            src/planning/plane.cpp
            src/planning/plane_solver_old.cpp
            src/planning/qpstructure.cpp
            src/planning/cubic_spline.cpp
            src/planning/motion_generator.cpp
            src/controller/controller.cpp
            src/controller/joint_velocity_example_controller.cpp)

target_include_directories(panda_mpc SYSTEM PUBLIC include
                                            ${Franka_INCLUDE_DIRS}
                                            ${GAZEBO_INCLUDE_DIRS}
                                            ${catkin_INCLUDE_DIRS}
                                            ${qpOASES_INCLUDE_DIRS} )

target_link_libraries(panda_mpc PUBLIC ${catkin_LIBRARIES}
                                       ${qpOASES_LIBRARIES}
                                       ${Boost_LIBRARIES}
                                       ${Franka_LIBRARIES}
                                       ${GAZEBO_LIBRARIES} )

ADD_SUBDIRECTORY(examples)

## Installation

install(TARGETS panda_mpc
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
        )

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(FILES panda_mpc_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(FILES scripts/killgazebo
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ WORLD_EXECUTE GROUP_EXECUTE OWNER_EXECUTE
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
