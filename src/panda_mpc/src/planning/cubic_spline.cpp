#include "planning/cubic_spline.h"

namespace cubicSpline {


bool cubicSpline::JntSpaceCubicInterpolation(const std::vector<KDL::JntArrayAcc> & jnt_points, int i){


  ROS_WARN_STREAM("jnt space cubic interpolation");
  Eigen::VectorXd jerk;
  jerk.resize(jnt_points.size());
  jerk.setConstant(200);
  A << 2*(dt_+ dt_), dt_, 0 ,
       dt_, 2*(dt_ + dt_), dt_,
       0, dt_, 2*(dt_+dt_);

  double q0,q1,q2,q3,q4;

  q0 = jnt_points[0].qdotdot(i);
  q1 = jnt_points[1].qdotdot(i);
  q2 = jnt_points[2].qdotdot(i);
  q3 = jnt_points[3].qdotdot(i);
  q4 = jnt_points[4].qdotdot(i);


  c << 3*(pow(dt_,2)*(q2-q1)+pow(dt_,2)*(q1-q0))/(dt_*dt_) - dt_*jerk(0),
       3*(pow(dt_,2)*(q3-q2)+pow(dt_,2)*(q2-q1))/(dt_*dt_),
      3*(pow(dt_,2)*(q4-q3)+pow(dt_,2)*(q3-q2))/(dt_*dt_) - dt_*jerk(0);

  v = A.inverse()*c;
  jerk.segment(1,v.size()) = v;

  for (int k(0); k < N_; k++){

//    std::cout <<" k " << k << '\n';
    coefficient_matrix_(k,0) = jnt_points[k].qdotdot(i); // The k-th via-point position of joint i
    coefficient_matrix_(k,1) = jerk(k);
    coefficient_matrix_(k,2) =(1/dt_) * (3*(jnt_points[k+1].qdotdot(i) - jnt_points[k].qdotdot(i))/dt_ - 2*jerk(k) - jerk(k+1));
    coefficient_matrix_(k,3) =(1/(dt_*dt_)) * (2*(jnt_points[k].qdotdot(i) - jnt_points[k+1].qdotdot(i))/dt_ +jerk(k) + jerk(k+1));

  }

  return true;
}


}
