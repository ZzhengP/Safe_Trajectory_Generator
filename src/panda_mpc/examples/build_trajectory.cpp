#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/path_point.hpp>
#include <kdl/path_line.hpp>
#include <panda_mpc/UpdateTrajectoryNextPoint.h>
#include <panda_mpc/trajectoryMsg.h>
#include <panda_mpc/trajectoryAcceleration.h>

#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/velocityprofile_spline.hpp>
#include <memory>
#include "ros/ros.h"
#include <nav_msgs/Path.h>
class TrajectoryBuilder{
public:
  TrajectoryBuilder(ros::NodeHandle & node_handle){

    path_sub_ = node_handle.subscribe("/mpc_path",1, &TrajectoryBuilder::trajBuilderCallback, this);
    next_traj_ = node_handle.advertise<panda_mpc::trajectoryAcceleration>("/mpc_solution",1);

  }

  void trajBuilderCallback(const nav_msgs::Path& path){

    KDL::Path_Line *path_line;

    KDL::VelocityProfile* velpref_;
    velpref_ = new KDL::VelocityProfile_Trap(1.5,12);
    KDL::Trajectory* traject_;
    Eigen::Vector3d err_goal_A, err_goal_B ;


      KDL::Rotation rot = KDL::Rotation::RPY(-M_PI,0,0);
      KDL::Vector vec(path.poses[0].pose.position.x,
                      path.poses[0].pose.position.y,
                      path.poses[0].pose.position.z);

      KDL::Frame frame(rot,vec);

      KDL::Vector new_vec(path.poses[1].pose.position.x,
                      path.poses[1].pose.position.y,
                      path.poses[1].pose.position.z);

      KDL::Frame new_frame(rot,new_vec);


      path_line = new KDL::Path_Line(frame, new_frame, new KDL::RotationalInterpolation_SingleAxis(), 0.01);


      velpref_->SetProfile(0, path_line->PathLength()); // Set velocity profile from start and end positions
      traject_ = (new KDL::Trajectory_Segment(path_line,velpref_));


    int i =0;
    double dt = 0.001;

    for (double t=0.; t < traject_->Duration(); t+=dt ){
      if (t > 0.05){
        break;
      }
      KDL::Frame desired_pose;
      desired_pose = traject_->Pos(t);

      panda_mpc::trajectoryAcceleration next_acceleration;
      next_acceleration.jntAcc.resize(6);
      next_acceleration.header.stamp = ros::Time::now();

      next_acceleration.jntAcc.at(0) = desired_pose.p.x();
      next_acceleration.jntAcc.at(1) = desired_pose.p.y();
      next_acceleration.jntAcc.at(2) = desired_pose.p.z();

      next_acceleration.jntAcc.at(3) = traject_->Vel(t).vel.x();
      next_acceleration.jntAcc.at(4) = traject_->Vel(t).vel.y();
      next_acceleration.jntAcc.at(5) = traject_->Vel(t).vel.z();

      next_traj_.publish(next_acceleration);
      i ++ ;
    }
    ROS_WARN_STREAM("traj count  " << i);

    path_line = nullptr;
    velpref_ = nullptr;
    traject_ = nullptr;
    ROS_WARN_STREAM("Trajec sender finish ");

  }


  private:

  ros::Subscriber path_sub_;
  ros::Publisher path_pub_;
  ros::Publisher next_traj_;

};

int main(int argc, char *argv[])
{

  ros::init(argc,argv, "TrajectoryBuilder");
  ros::NodeHandle node_handle;
  double dt;
  node_handle.getParam("/panda_mpc/dt_", dt);

  int rate = 1000;
  ros::Rate loop_rate(rate);

  TrajectoryBuilder trajec_builder(node_handle);
  while(ros::ok()){
      ros::spinOnce();
      loop_rate.sleep();

  }

  return 0;
}
