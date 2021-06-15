#ifndef PLANE_H
#define PLANE_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Core>
#include <memory>
#include <qpOASES.hpp>
#include <std_msgs/Float64MultiArray.h>
//#include <planning/plane_solver.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <planning/plane_solver_old.h>

namespace planning {

/**
 * @brief msgToEigen: convert ros multiarray message to eigen matrix
 * @param msg
 * @return eigen matrixXd
 */
Eigen::MatrixXd msgToEigen(const std_msgs::Float64MultiArray::ConstPtr& msg);

Eigen::MatrixXd ComputeObstacleVertices(Eigen::Vector3d centre, Eigen::Vector3d size);
std::pair<int,int> closestPoint(Eigen::MatrixXd robotVertices, Eigen::MatrixXd obsVertices);


class plane{

public:

  plane(ros::NodeHandle& node_handle, int N, int nbrCst, double dsafe);

  bool init(const std::vector<Eigen::MatrixXd> &robot_vertices_horizon,
            const std::vector<Eigen::MatrixXd>& obs_vertices_horizon);

  bool update(const std::vector<Eigen::MatrixXd>& robot_vertices_horizon,
              const std::vector<Eigen::MatrixXd>& obstacle_vertices_horizon);

  Eigen::MatrixXd GetFirstPlane() const {
      return plane_location_[0];
  }

   std::vector<Eigen::MatrixXd> GetPlane() const{
    return plane_location_;
  }
private:

  PlaneSolver plane_solver_;
  int N_;
  int nbr_of_plane_ ; /*!< @brief The total number of plane */
  int nbr_of_robotPart_; /*!< @brief robot number of link */
  int nbr_of_obsPart_; /*!< @brief Obstacle number  */

  int nbr_of_robot_vertices_; /*!< @brief robot vertices for one member */
  int nbr_of_obstacle_vertices_; /*!< @brief obstacle vertices for one member */

  int nbrCst_;
  double dsafe_;

  std::vector<Eigen::MatrixXd> robot_vertices_; /*!< @brief robot number of vertices */
  std::vector<Eigen::MatrixXd> obstacle_vertices_; /*!< @brief obstacle number of vertices */

  std::vector<Eigen::MatrixXd> robot_vertices_horizon_;/*!< @brief differents part of robot vertices put into one different matrix */
  std::vector<Eigen::MatrixXd> obstacle_vertices_horizon_; /*!< @brief differents part of obstacle vertices put into one different matrix */
  std::vector<Eigen::MatrixXd> plane_location_, plane_precedent_;

  Eigen::Vector3d obs_center_;
  // Plane visualization
  rviz_visual_tools::RvizVisualTools rviz_planes_publisher_,rviz_obstacle_publisher_;
  rviz_visual_tools::colors color_plane_ = rviz_visual_tools::TRANSLUCENT_DARK;

  bool publishABCDPlane(double A, double B, double C, double D, rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT,
                        double x_width = 2.0, double y_width = 2.0);
};


}


#endif // PLANE_H
