#include "o2ac_pose_distribution_updater/pose_belief_visualizer.hpp"

void convert_to_triangle_list(
    const moveit_msgs::CollisionObject &object,
    std::vector<geometry_msgs::Point> &triangle_list) {
  // convert from moveit_msgs::Collisionobject to the concatenat list of the
  // vertices of triangles

  // first, convert to Eigen vectors and triplets of vertex ids.
  std::vector<Eigen::Vector3d> vertices;
  std::vector<boost::array<int, 3>> triangles;
  CollisionObject_to_eigen_vectors(object, vertices, triangles);

  std::vector<geometry_msgs::Point> msg_vertices(vertices.size());

  // convert vertices from Eigen::Vector3d to geometry_msgs::Point
  for (int i = 0; i < vertices.size(); i++) {
    tf::pointEigenToMsg(vertices[i], msg_vertices[i]);
  }

  // make triangle_list
  for (auto &triangle : triangles) {
    for (int k = 0; k < 3; k++) {
      triangle_list.push_back(msg_vertices[triangle[k]]);
    }
  }
}

std::random_device seed_generator;
std::default_random_engine engine(seed_generator());
std::normal_distribution<> unit_normal_distribution(0.0, 1.0);

void PoseBeliefVisualizer::make_marker_from_particle(
    const std_msgs::Header &header,
    const std::vector<geometry_msgs::Point> &triangle_list,
    const geometry_msgs::Pose &pose, const std_msgs::ColorRGBA &color,
    const ros::Duration &lifetime, visualization_msgs::Marker &marker) {
  // Given header, object shape given as triangle list, pose given as
  // geometry_msgs::pose and color, make a marker

  marker.header = header;
  marker.ns = object_namespace;
  marker.id = object_id++;

  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.points = triangle_list;

  marker.pose = pose;
  marker.scale = scale;
  marker.color = color;
  marker.lifetime = lifetime;

  marker.frame_locked = true;
  marker.action = visualization_msgs::Marker::ADD;
}

void PoseBeliefVisualizer::inner_publish_marker_for_pose_belief(
    o2ac_msgs::visualizePoseBelief::Request &belief,
    o2ac_msgs::visualizePoseBelief::Response &response) {
  // this procedure publishes the marker array corresponding to the given pose
  // belief

  // convert belief.gripped_object from movei_msgs::Collisionobject to
  // triangle list
  std::vector<geometry_msgs::Point> triangle_list;
  convert_to_triangle_list(belief.object, triangle_list);

  // convert geometry_msgs::PoseWithCovariance to CovarianceMatrix
  CovarianceMatrix covariance =
      array_36_to_matrix_6x6(belief.distribution.pose.covariance);

  // Store poses and their colors to visualize
  std::vector<std::pair<geometry_msgs::Pose, std_msgs::ColorRGBA>>
      poses_to_publish;

  // mean and other 4 poses corresponding to the first two components of
  // covariance is displayed

  poses_to_publish.push_back(
      std::make_pair(belief.distribution.pose.pose, mean_color));

  CovarianceMatrix Cholesky_L(
      covariance.llt().matrixL()); // old_covariance == Cholesky_L *
  // Cholesky_L.transpose()

  if (belief.distribution_type == belief.RPY_COVARIANCE) {
    // The covariance matrix is interpreted as covariance in 6 axes of particles

    Particle mean = pose_to_particle(belief.distribution.pose.pose);
    for (int i = 0; i < number_of_particles; i++) {
      Particle particle = mean + Cholesky_L * get_UND_particle();
      geometry_msgs::Pose converted_pose;
      particle_to_pose(particle, converted_pose);
      poses_to_publish.push_back(
          std::make_pair(converted_pose, variance_color));
    }
  }

  if (belief.distribution_type == belief.LIE_COVARIANCE) {
    // The covariance matrix is interpreted as covariance in Lie algebra

    Eigen::Isometry3d mean;
    tf::poseMsgToEigen(belief.distribution.pose.pose, mean);
    for (int i = 0; i < number_of_particles; i++) {
      Eigen::Matrix<double, 6, 1> deviation_vector =
          Cholesky_L * get_UND_particle();
      Eigen::Isometry3d deviation =
          Eigen::Isometry3d(Eigen::Matrix<double, 4, 4>(
              hat_operator<double>(deviation_vector).exp())) *
          mean;
      // convert this transform to msg poses and store them
      geometry_msgs::Pose converted_pose;
      tf::poseEigenToMsg(deviation, converted_pose);
      poses_to_publish.push_back(
          std::make_pair(converted_pose, variance_color));
    }
  }

  // make visualization markers
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(poses_to_publish.size());
  for (int i = 0; i < poses_to_publish.size(); i++) {
    make_marker_from_particle(
        belief.distribution.header, triangle_list, poses_to_publish[i].first,
        poses_to_publish[i].second, belief.lifetime, marker_array.markers[i]);
  }

  // publish visualization markers
  marker_publisher.publish(marker_array);
}
