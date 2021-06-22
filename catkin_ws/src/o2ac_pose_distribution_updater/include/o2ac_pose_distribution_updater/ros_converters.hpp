#include <eigen_conversions/eigen_msg.h>
#include <fcl/math/transform.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32MultiArray.h>

#include "o2ac_pose_distribution_updater/conversions.hpp"

fcl::Transform3f pose_to_fcl_transform(const geometry_msgs::Pose &pose);

Particle pose_to_particle(const geometry_msgs::Pose &pose);

CovarianceMatrix array_36_to_matrix_6x6(const boost::array<double, 36> &array);

boost::array<double, 36> matrix_6x6_to_array_36(const CovarianceMatrix &matrix);

geometry_msgs::Pose to_Pose(const double &x, const double &y, const double &z,
                            const double &qw, const double &qx,
                            const double &qy, const double &qz);

void particle_to_pose(const Particle &particle, geometry_msgs::Pose &pose);

geometry_msgs::PoseWithCovariance
to_PoseWithCovariance(const Particle &mean, const CovarianceMatrix &covariance);

void CollisionObject_to_eigen_vectors(
    const moveit_msgs::CollisionObject &object,
    std::vector<Eigen::Vector3d> &vertices,
    std::vector<boost::array<int, 3>> &triangles);

void add_mesh_to_CollisionObject(
    std::shared_ptr<moveit_msgs::CollisionObject> &object,
    const std::vector<Eigen::Vector3d> &vertices,
    const std::vector<boost::array<int, 3>> &triangles,
    const Eigen::Isometry3d &transform);

void msg_pose_to_msg_transform(const geometry_msgs::Pose &pose,
                               geometry_msgs::Transform &transform);
