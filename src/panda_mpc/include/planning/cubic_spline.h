#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H

#include "Eigen/Core"
#include <Eigen/Dense>
#include "ros/ros.h"
#include "kdl/jntarrayvel.hpp"
#include "kdl/jntarrayacc.hpp"

namespace cubicSpline {

class cubicSpline{

public:

  cubicSpline(int N, double dt):
              N_{N}, dt_{dt}{


   coefficient_matrix_.resize(N_, 4); // Cubic split -> 4 parameters for each segment

   A.resize(N_-1,N_-1);
   c.resize(N_-1);
   v.resize(N_ - 1);
  }

  /**
   * @brief JntSpaceCubicInterpolation: multi-points cubic spline interpolation for joint with index i
   * @param jnt_points: a set of joint, start to end with multi via-points
   * @param index:
   * @return
   */
  bool JntSpaceCubicInterpolation(const std::vector<KDL::JntArrayAcc> & jnt_points, int i );

  Eigen::MatrixXd getCoefficientMatrix(){
    return coefficient_matrix_;
  }
private:

  int N_; // Number of via-points
  double dt_; // constant split time

  Eigen::MatrixXd coefficient_matrix_;
  Eigen::MatrixXd A;
  Eigen::VectorXd c;
  Eigen::VectorXd v;
};

}
#endif // CUBIC_SPLINE_H
