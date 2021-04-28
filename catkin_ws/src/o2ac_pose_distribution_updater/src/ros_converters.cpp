/*
Convertion functions associated to geometry_msgs types
 */

#include <fcl/math/transform.h>
#include <Eigen/Geometry>

// Note that Particle is Eigen::Matrix<double, 6, 1> and CovarianceMatrix is Eigen::Matrix<double, 6, 6>

fcl::Transform3f pose_to_fcl_transform(const geometry_msgs::Pose &pose)
{
  return fcl::Transform3f(fcl::Quaternion3f(pose.orientation.w, pose.orientation.x, pose.orientation.y,pose.orientation.z),
			  fcl::Vec3f(pose.position.x, pose.position.y, pose.position.z));
}

Eigen::Transform<double, 3, Eigen::Isometry> pose_to_eigen_transform(const geometry_msgs::Pose &pose)
{
  Eigen::Vector3d translation;
  translation << pose.position.x, pose.position.y, pose.position.z;
  return Eigen::Translation3d(translation)
    * Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y,pose.orientation.z);
}

Particle pose_to_particle(const geometry_msgs::Pose &pose)
{
  Particle particle;
  double roll, pitch, yaw;
  quaternion_to_RPY(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z, roll, pitch, yaw);
  particle << pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw;
  return particle;
}

CovarianceMatrix array_36_to_matrix_6x6(const boost::array<double, 36> &array)
{
  CovarianceMatrix matrix;
  for(int i=0; i<6 ; i++){
    for(int j=0; j<6 ; j++){
      matrix(i, j) = array[6 * i + j];
    }
  }
  return matrix;
}

geometry_msgs::Pose to_Pose(const double &x, const double &y, const double &z, const double &qw, const double &qx, const double &qy, const double &qz)
{
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = qw;
  pose.orientation.x = qx;
  pose.orientation.y = qy;
  pose.orientation.z = qz;
  return pose;
}

geometry_msgs::PoseWithCovariance to_PoseWithCovariance(const Particle &mean, const CovarianceMatrix &covariance)
{
  geometry_msgs::PoseWithCovariance pwc;
  pwc.pose.position.x = mean(0);
  pwc.pose.position.y = mean(1);
  pwc.pose.position.z = mean(2);
  RPY_to_quaternion(mean(3), mean(4), mean(5), pwc.pose.orientation.w, pwc.pose.orientation.x, pwc.pose.orientation.y, pwc.pose.orientation.z);
  for(int i=0; i<6; i++){
    for(int j=0; j<6; j++){
      pwc.covariance[6 * i + j] = covariance(i, j);
    }
  }
  return pwc;
}

