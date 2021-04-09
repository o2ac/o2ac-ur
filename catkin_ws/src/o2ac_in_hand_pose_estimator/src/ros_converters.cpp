fcl::Transform3f pose_to_transform(const geometry_msgs::Pose &pose)
{
  return fcl::Transform3f(fcl::Quaternion3f(pose.orientation.w, pose.orientation.x, pose.orientation.y,pose.orientation.z),
			  fcl::Vec3f(pose.position.x, pose.position.y, pose.position.z));
}

Particle pose_to_particle(const geometry_msgs::Pose &pose)
{
  Particle particle;
  Real roll, pitch, yaw;
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

geometry_msgs::Pose to_Pose(double x, double y, double z, double qw, double qx, double qy, double qz)
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

