#include <planning/plane.h>

namespace planning {

plane::plane(ros::NodeHandle& node_handle, int N, int nbrCst, double dsafe):N_{N},nbrCst_{nbrCst},dsafe_{dsafe},
  plane_solver_{nbrCst,dsafe},rviz_planes_publisher_{"panda_link0","/separating_plane"},rviz_obstacle_publisher_{"panda_link0","/obstacle"}
{

 ROS_WARN_STREAM("Plane Constructor");
  //
 node_handle.getParam("/panda_mpc/robot_member_", nbr_of_robotPart_);
 node_handle.getParam("/panda_mpc/obstacle_member_", nbr_of_obsPart_);
 node_handle.getParam("/panda_mpc/robot_vertices_", nbr_of_robot_vertices_);
 node_handle.getParam("/panda_mpc/obstacle_vertices_", nbr_of_obstacle_vertices_);

 // Initialize robot vertices data and associated data on horizon
 robot_vertices_horizon_.resize(nbr_of_robotPart_);
 robot_vertices_.resize(nbr_of_robotPart_);



  obstacle_vertices_.resize(nbr_of_obsPart_);




  nbr_of_plane_ = nbr_of_obsPart_;
  plane_location_.resize(nbr_of_plane_);

  plane_precedent_.resize(nbr_of_plane_);

  // Initialize plane solver
//  plane_solver_.init();

  ROS_WARN_STREAM("Plane objet successufily create");

}

bool plane::init(const std::vector<Eigen::MatrixXd> &robot_vertices_horizon,
                 const std::vector<Eigen::MatrixXd>& obs_vertices_horizon){

  ROS_WARN_STREAM("Plane initializing");


  // Fill robot vertices data
  robot_vertices_horizon_.resize(robot_vertices_horizon.size());
  for (int i(0); i < robot_vertices_horizon_.size(); i++){
    robot_vertices_horizon_[i].resize(robot_vertices_horizon[i].rows(),robot_vertices_horizon[i].cols());
    robot_vertices_horizon_[i] = robot_vertices_horizon[i];
  }

  obstacle_vertices_horizon_.resize(obs_vertices_horizon.size());
  for (int i(0); i < obstacle_vertices_horizon_.size(); i++){
    obstacle_vertices_horizon_[i].resize(obs_vertices_horizon[i].rows(),obs_vertices_horizon[i].cols());
    obstacle_vertices_horizon_[i] = obs_vertices_horizon[i];
  }

  // Fill obstacle vertices data
  for (int i(0); i<nbr_of_obsPart_;i++){
    obstacle_vertices_[i].resize(3,nbr_of_obstacle_vertices_);
    obstacle_vertices_[i].block(0,0,3,nbr_of_obstacle_vertices_) = obs_vertices_horizon[i].block(0,0,3,nbr_of_obstacle_vertices_);

   }
  // Fill plane data
  for (int i(0); i < nbr_of_plane_;i++){

    N_==1 ? plane_location_[i].resize(5,N_):plane_location_[i].resize(5,N_-1);
    N_==1 ? plane_precedent_[i].resize(5,N_):plane_precedent_[i].resize(5,N_-1);
    for (int j(0); j<N_-1; j++){
      plane_location_[i].block(0,j,5,1) << 0,
                                           -1,
                                           0.2,
                                           0.,
                                           0;

    }
    plane_precedent_[i] = plane_location_[i];

  }

  ROS_WARN_STREAM("Plane initialized");

  return true;
}

bool plane::update(const std::vector<Eigen::MatrixXd> &robot_vertices_horizon,
                   const std::vector<Eigen::MatrixXd> &obstacle_vertices_horizon){

//   rviz_planes_publisher_.deleteAllMarkers();
//   rviz_obstacle_publisher_.deleteAllMarkers();

   rviz_visual_tools::colors color = rviz_visual_tools::BLUE;
//   rviz_obstacle_publisher_.publishSphere(obs_center_,
//                                           color,
//                                           0.1);
//   rviz_obstacle_publisher_.trigger();


   bool is_plane_solved = false;
  // Update robot vertices data
  for(size_t i(0); i < robot_vertices_horizon.size(); i++ ){
    robot_vertices_horizon_[i] = robot_vertices_horizon[i];
  }

  // Update obstacle vertices data
  for(size_t i(0); i < obstacle_vertices_horizon.size(); i++){
    obstacle_vertices_horizon_[i] = obstacle_vertices_horizon[i];
  }

  for (int i(0); i < nbr_of_obsPart_; i++){
    for (int j(0); j < nbr_of_robotPart_; j++) {
      Eigen::VectorXd plane_solution;
      plane_solution.resize(5);
      for (int k(0); k < N_-1 ; k++){

         plane_precedent_[0].block(0,k,5,1)= plane_location_[0].block(0,k,5,1);
//         plane_solution  =  plane_solver_.update(plane_precedent_[0].block(0,k,5,1), robot_vertices_horizon_[0].block(0,k,3,2*nbr_of_robot_vertices_),
//                           obstacle_vertices_horizon_[0].block(0,k*nbr_of_obstacle_vertices_,3,2*nbr_of_obstacle_vertices_));
         plane_solver_.setCost(plane_location_[0].block(0,k,5,1));
         plane_solver_.setCstMatrix(robot_vertices_horizon_[0].block(0,k,3,2*nbr_of_robot_vertices_),
                                    obstacle_vertices_horizon_[0].block(0,k*nbr_of_obstacle_vertices_,3,2*nbr_of_obstacle_vertices_),
                                    plane_location_[0].block(0,k,5,1));

          is_plane_solved = plane_solver_.solve();

            plane_location_[0].block(0,k,5,1) = plane_solver_.getSolution();
            plane_location_[0].block(0,k,3,1) = plane_location_[0].block(0,k,3,1)/plane_location_[0].block(0,k,3,1).norm();
            plane_location_[0](3,k) = plane_location_[0](3,k)*plane_location_[0].block(0,k,3,1).norm();

         if (!is_plane_solved){
           ROS_WARN_STREAM("plane solution failed");

         }else{
//           plane_location_[0].block(0,k,5,1) = plane_solver_.getSolution();
         }
      }
    }
  }


  return true;
}

Eigen::MatrixXd msgToEigen(const std_msgs::Float64MultiArray::ConstPtr& msg){

  Eigen::MatrixXd eigenMatrix;
  int rows, cols, rows_stride, cols_stride;

  rows = msg->layout.dim[0].size;
  cols = msg->layout.dim[1].size;

  rows_stride = msg->layout.dim[0].stride;
  cols_stride = msg->layout.dim[1].stride;

  eigenMatrix.resize(rows, cols);
  eigenMatrix.setZero();

  int ii=0;

  for (int i=0; i < rows; i++){
    for (int j =0; j<cols; j++){
      eigenMatrix(i,j) = msg->data[ii++];
    }
  }

  return eigenMatrix;
}


Eigen::MatrixXd ComputeObstacleVertices(Eigen::Vector3d centre, Eigen::Vector3d size){

  Eigen::MatrixXd vertices;
  vertices.resize(3,8);

  vertices.block(0,0,1,8)<< centre(0) - size(0), centre(0) - size(0), centre(0) - size(0),centre(0) - size(0),
                            centre(0) + size(0), centre(0) + size(0), centre(0) + size(0),centre(0) + size(0);

  vertices.block(1,0,1,8)<< centre(1) + size(1), centre(1) + size(1), centre(1) - size(1),centre(1) - size(1),
                            centre(1) + size(1), centre(1) + size(1), centre(1) - size(1),centre(1) - size(1);

  vertices.block(2,0,1,8)<< centre(2) - size(2), centre(2) + size(2), centre(2) - size(2),centre(2) + size(2),
                            centre(2) - size(2), centre(2) + size(2), centre(2) - size(2),centre(2) + size(2);

  return vertices;
}

std::pair<int,int> closestPoint(Eigen::MatrixXd robotVertices, Eigen::MatrixXd obsVertices){

  std::pair<int, int> indexClosestPts;
  double dmin = 20;

  for (int i(0); i < robotVertices.cols(); i++){
    for (int j = 0; j<obsVertices.cols(); j++){

      Eigen::Vector3d dist_temp;
      dist_temp = robotVertices.block(0,i,3,1) - obsVertices.block(0,j,3,1);
      if(dist_temp.norm() < dmin){
        dmin = dist_temp.norm();
        indexClosestPts.first=i;
        indexClosestPts.second=j;
      }
    }
  }

  return indexClosestPts;
}

//bool plane::publishABCDPlane(double A, double B, double C, double D, rviz_visual_tools::colors color,
//                      double x_width, double y_width ){

//    Eigen::Vector3d n(A,B,C);
//    double distance = D/n.norm();
//    Eigen::Vector3d center = -distance*n.normalized();

//    Eigen::Isometry3d pose;
//    pose.translation() = center;

//    Eigen::Vector3d z_0 = Eigen::Vector3d::UnitZ();
//    Eigen::Quaterniond q= Eigen::Quaterniond::FromTwoVectors(z_0,n);
//    pose.linear() = q.toRotationMatrix();

//    double height = 0.001;

//    rviz_planes_publisher_.publishCuboid(pose, x_width,y_width,height,color);

//    rviz_planes_publisher_.trigger();
//    return true;
//}
}
