/*
Convertion functions associated to geometry_msgs types
 */

#include "o2ac_pose_distribution_updater/ros_converters.hpp"

// Note that Particle is Eigen::Matrix<double, 6, 1> and CovarianceMatrix is
// Eigen::Matrix<double, 6, 6>

fcl::Transform3f pose_to_fcl_transform(const geometry_msgs::Pose &pose) {
  return fcl::Transform3f(
      fcl::Quaternion3f(pose.orientation.w, pose.orientation.x,
                        pose.orientation.y, pose.orientation.z),
      fcl::Vec3f(pose.position.x, pose.position.y, pose.position.z));
}

Particle pose_to_particle(const geometry_msgs::Pose &pose) {
  Particle particle;
  double roll, pitch, yaw;
  quaternion_to_RPY(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, roll, pitch, yaw);
  particle << pose.position.x, pose.position.y, pose.position.z, roll, pitch,
      yaw;
  return particle;
}

CovarianceMatrix array_36_to_matrix_6x6(const boost::array<double, 36> &array) {
  CovarianceMatrix matrix;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      matrix(i, j) = array[6 * i + j];
    }
  }
  return matrix;
}

geometry_msgs::Pose to_Pose(const double &x, const double &y, const double &z,
                            const double &qw, const double &qx,
                            const double &qy, const double &qz) {
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

geometry_msgs::PoseWithCovariance
to_PoseWithCovariance(const Particle &mean,
                      const CovarianceMatrix &covariance) {
  geometry_msgs::PoseWithCovariance pwc;
  pwc.pose.position.x = mean(0);
  pwc.pose.position.y = mean(1);
  pwc.pose.position.z = mean(2);
  RPY_to_quaternion(mean(3), mean(4), mean(5), pwc.pose.orientation.w,
                    pwc.pose.orientation.x, pwc.pose.orientation.y,
                    pwc.pose.orientation.z);
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      pwc.covariance[6 * i + j] = covariance(i, j);
    }
  }
  return pwc;
}

void CollisionObject_to_eigen_vectors(
    const moveit_msgs::CollisionObject &object,
    std::vector<Eigen::Vector3d> &vertices,
    std::vector<boost::array<int, 3>> &triangles) {
  Eigen::Isometry3d object_transform;
  tf::poseMsgToEigen(object.pose, object_transform);
  for (int mesh_id = 0; mesh_id < object.meshes.size(); mesh_id++) {
    int current_size = vertices.size();
    auto &mesh = object.meshes[mesh_id];
    Eigen::Isometry3d mesh_transform;
    tf::poseMsgToEigen(object.mesh_poses[mesh_id], mesh_transform);
    auto transform = object_transform * mesh_transform;
    for (auto &vertex : mesh.vertices) {
      Eigen::Vector3d eigen_vertex;
      tf::pointMsgToEigen(vertex, eigen_vertex);
      vertices.push_back(transform * eigen_vertex);
    }
    for (auto &triangle : mesh.triangles) {
      boost::array<int, 3> vertex_indices;
      for (int k = 0; k < 3; k++) {
        vertex_indices[k] = current_size + triangle.vertex_indices[k];
      }
      triangles.push_back(vertex_indices);
    }
  }
}

void add_mesh_to_CollisionObject(
    std::shared_ptr<moveit_msgs::CollisionObject> &object,
    const std::vector<Eigen::Vector3d> &vertices,
    const std::vector<boost::array<int, 3>> &triangles,
    const Eigen::Isometry3d &transform) {
  shape_msgs::Mesh mesh;
  mesh.vertices = std::vector<geometry_msgs::Point>(vertices.size());
  for (int i = 0; i < vertices.size(); i++) {
    tf::pointEigenToMsg(vertices[i], mesh.vertices[i]);
  }
  mesh.triangles = std::vector<shape_msgs::MeshTriangle>(triangles.size());
  for (int j = 0; j < triangles.size(); j++) {
    for (int k = 0; k < 3; k++) {
      mesh.triangles[j].vertex_indices[k] = triangles[j][k];
    }
  }
  object->meshes.push_back(mesh);
  geometry_msgs::Pose pose;
  tf::poseEigenToMsg(transform, pose);
  object->mesh_poses.push_back(pose);
}
