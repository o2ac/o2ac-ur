/*
Definitions of type and conversion functions associated to quaternion
 */

#include <Eigen/Core>

using Particle = Eigen::Matrix<double, 6, 1>;
using CovarianceMatrix  = Eigen::Matrix<double, 6, 6>;

template<typename Scalar> void RPY_to_quaternion(const Scalar &roll, const Scalar &pitch, const Scalar &yaw, Scalar &w, Scalar &x, Scalar &y, Scalar &z){
  Scalar sr=std::sin(roll / 2.0);
  Scalar cr=std::cos(roll / 2.0);
  Scalar sp=std::sin(pitch / 2.0);
  Scalar cp=std::cos(pitch / 2.0);
  Scalar sy=std::sin(yaw / 2.0);
  Scalar cy=std::cos(yaw / 2.0);
  w = cr * cp * cy + sr * sp * sy;
  x = sr * cp * cy - cr * sp * sy;
  y = cr * sp * cy + sr * cp * sy;
  z = cr * cp * sy - sr * sp * cy;
}

template<typename Scalar> void quaternion_to_RPY(const Scalar &w, const Scalar &x, const Scalar &y, const Scalar &z, Scalar &roll, Scalar &pitch, Scalar &yaw){
  using namespace std;
  roll = atan2(2.0*(w*x+y*z), w*w-x*x-y*y+z*z);
  pitch = asin(2.0*(w*y-x*z));
  yaw = atan2(2.0*(w*z+x*y), w*w+x*x-y*y-z*z);
}
