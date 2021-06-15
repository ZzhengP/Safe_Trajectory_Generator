#include <iostream>
#include <qpOASES.hpp>
#include <Eigen/Core>
#include <planning/plane_solver.h>
#include <planning/plane.h>
#include "ros/ros.h"

Eigen::MatrixXd ComputeObstacleVertices(Eigen::Vector3d centre, Eigen::Vector3d size){

    Eigen::MatrixXd Vertices;
    Vertices.resize(3,8);

    Vertices.block(0,0,1,8) << centre(0) - size(0), centre(0) - size(0), centre(0) - size(0), centre(0) - size(0),
                               centre(0) + size(0), centre(0) + size(0), centre(0) + size(0), centre(0) + size(0);

    Vertices.block(1,0,1,8) << centre(1) + size(1), centre(1) + size(1), centre(1) - size(1), centre(1) - size(1),
                               centre(1) + size(1), centre(1) + size(1), centre(1) - size(1), centre(1) - size(1);

    Vertices.block(2,0,1,8) << centre(2) - size(2), centre(2) + size(2), centre(2) - size(2), centre(2) + size(2),
                               centre(2) - size(2), centre(2) + size(2), centre(2) - size(2), centre(2) + size(2);

    return  Vertices;
}



int main(int argc, char** argv){

  ros::init(argc,argv, "separating_plane");
  ros::NodeHandle node_handle;
  ros::Rate loop_rate(50);

  Eigen::Vector3d obsSize, obsCentre;

  obsSize << 0.05, 0.05, 0.05;
  obsCentre << 1.0, 0., 0.1;

  int N = 5;
  node_handle.getParam("/panda_mpc/N_", N);

  double dsafe = 0.2;
  int nbr_of_obstacle = 1;
  int nbr_of_obstacle_vertices = 1;
  node_handle.getParam("/panda_mpc/obstacle_vertices_", nbr_of_obstacle_vertices);

  Eigen::MatrixXd obsVertices;
  std::vector<Eigen::MatrixXd> obsVerticesAugmented;

  obsVertices.resize(3*nbr_of_obstacle,nbr_of_obstacle_vertices);
//  obsVertices = ComputeObstacleVertices(obsCentre,obsSize);
  obsVertices << obsCentre;


  obsVerticesAugmented.resize(nbr_of_obstacle);
  for (int j(0); j < nbr_of_obstacle; j++){

     obsVerticesAugmented[j].resize(3,nbr_of_obstacle_vertices*N);
     for (int i(0) ; i < N ; i++) {
         obsVerticesAugmented[j].block(0,i*nbr_of_obstacle_vertices,3,nbr_of_obstacle_vertices) = obsVertices;
     }
  }

  Eigen::MatrixXd robotVertices;
  std::vector<Eigen::MatrixXd> robotVerticesAugmented;
  int nbr_of_robot_vertices = 1;
  robotVertices.resize(3,nbr_of_robot_vertices);
  robotVertices << 0.5, 0., 0.4;
  robotVerticesAugmented.resize(1);
  robotVerticesAugmented[0].resize(3,N);

  for (int i(0); i < N; i++){
    robotVerticesAugmented[0].block(0,i,3,1) = robotVertices;
  }

  int nbr_of_constraint = 10;
  planning::plane plane(node_handle,N,nbr_of_constraint,dsafe);

  if(!plane.init(robotVerticesAugmented,obsVerticesAugmented))
    ROS_WARN_STREAM("Plane initialization failed ");
  int count = 0;
  while (ros::ok()){


    plane.update(robotVerticesAugmented,
                  obsVerticesAugmented);


    count ++;
    ROS_WARN_STREAM("count" << count);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
