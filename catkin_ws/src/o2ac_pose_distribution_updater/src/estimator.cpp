/*
The implementation of estimator, which calculates the updated distribution for
each action
 */

#include "o2ac_pose_distribution_updater/estimator.hpp"

// Conversion functions

// Note that Particle is Eigen::Matrix<double, 6, 1> and CovarianceMatrix is
// Eigen::Matrix<double, 6, 6>

fcl::Transform3f particle_to_transform(const Particle &p) {
  double w, x, y, z;
  RPY_to_quaternion(p(3), p(4), p(5), w, x, y, z);
  return fcl::Transform3f(fcl::Quaternion3f(w, x, y, z),
                          fcl::Vec3f(p(0), p(1), p(2)));
}

Eigen::Transform<double, 3, Eigen::Isometry>
particle_to_eigen_transform(const Particle &p) {
  return Eigen::Translation3d(p.block(0, 0, 3, 1)) *
         Eigen::AngleAxis<double>(p(5), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxis<double>(p(4), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxis<double>(p(3), Eigen::Vector3d::UnitX());
}

Eigen::Vector3d to_eigen_vector(const fcl::Vec3f &v) {
  Eigen::Vector3d ev;
  ev << v.data[0], v.data[1], v.data[2];
  return ev;
}

Eigen::Transform<double, 3, Eigen::Isometry>
to_eigen_transform(const fcl::Transform3f &t) {
  return Eigen::Translation3d(to_eigen_vector(t.getTranslation())) *
         Eigen::Quaterniond(
             t.getQuatRotation().getW(), t.getQuatRotation().getX(),
             t.getQuatRotation().getY(), t.getQuatRotation().getZ());
}

std::random_device seed_generator;
std::default_random_engine engine(seed_generator());
std::normal_distribution<> unit_normal_distribution(0.0, 1.0);

Particle get_UND_particle() {
  // Random Particle generator

  // All coordinates are independent
  // The distribution of each coordinate is the normal distribution with mean
  // 0.0, variance 1.0

  Particle p;
  for (int i = 0; i < 6; i++) {
    p(i) = unit_normal_distribution(engine);
  }
  return p;
}

double calculate_distance(
    const std::shared_ptr<fcl::CollisionObject> &touched_object,
    const std::shared_ptr<fcl::CollisionGeometry> &gripped_geometry,
    const fcl::Transform3f &gripped_transform) {
  // Calculate the distance between 'touched_object' and 'gripped_geometry'
  // transfromed by 'gripped_transform'

  std::shared_ptr<fcl::CollisionObject> moved_object(
      new fcl::CollisionObject(gripped_geometry, gripped_transform));
  fcl::DistanceRequest distance_request;
  fcl::DistanceResult distance_result;
  double distance = fcl::distance(moved_object.get(), touched_object.get(),
                                  distance_request, distance_result);
  return distance;
}

Eigen::Vector3d calculate_center_of_gravity(
    const std::vector<Eigen::Vector3d> &vertices,
    const std::vector<boost::array<int, 3>> &triangles) {
  // Calculate the center of gravity

  Eigen::Vector3d center_of_gravity;
  center_of_gravity.setZero();
  double total_volume = 0.0;
  for (int j = 0; j < triangles.size(); j++) {
    auto ver0 = vertices[triangles[j][0]];
    auto ver1 = vertices[triangles[j][1]];
    auto ver2 = vertices[triangles[j][2]];
    double tetra_volume = ver0.dot(ver1.cross(ver2));
    center_of_gravity += tetra_volume * (ver0 + ver1 + ver2);
    total_volume += tetra_volume;
  }
  center_of_gravity /= 4.0 * total_volume;
  return center_of_gravity;
}

void PoseEstimator::set_particle_parameters(const int &number_of_particles,
                                            const Particle &noise_variance) {
  this->number_of_particles = number_of_particles;
  this->noise_variance = noise_variance;

  particles = std::vector<Particle>(number_of_particles);
  likelihoods = std::vector<double>(number_of_particles);
}

void PoseEstimator::set_touch_parameters(
    const std::vector<std::shared_ptr<fcl::CollisionObject>> &touched_objects,
    const double &distance_threshold) {
  this->touched_objects = touched_objects;
  this->distance_threshold = distance_threshold;
}

void PoseEstimator::set_look_parameters(
    const double &look_threshold,
    const std::vector<std::vector<double>> calibration_object_points,
    const std::vector<std::vector<double>> calibration_image_points,
    const double &camera_fx, const double &camera_fy, const double &camera_cx,
    const double &camera_cy) {
  this->look_threshold = look_threshold;
  std::vector<cv::Point3d> cv_calibration_object_points(
      calibration_object_points.size());
  std::vector<cv::Point2d> cv_calibration_image_points(
      calibration_image_points.size());
  for (int i = 0; i < calibration_object_points.size(); i++) {
    auto p = calibration_object_points[i];
    cv_calibration_object_points[i] = cv::Point3d(p[0], p[1], p[2]);
  }
  for (int i = 0; i < calibration_image_points.size(); i++) {
    auto p = calibration_image_points[i];
    cv_calibration_image_points[i] = cv::Point2d(p[0], p[1]);
  }
  camera_matrix.at<double>(0, 0) = camera_fx;
  camera_matrix.at<double>(1, 1) = camera_fy;
  camera_matrix.at<double>(0, 2) = camera_cx;
  camera_matrix.at<double>(1, 2) = camera_cy;

  // calculate the pose of camera, which is represented by 'camera_r' and
  // 'camera_t', using calibration points
  cv::solvePnP(cv_calibration_object_points, cv_calibration_image_points,
               camera_matrix, camera_dist_coeffs, camera_r, camera_t);
}

void PoseEstimator::generate_particles(const Particle &old_mean,
                                       const CovarianceMatrix &old_covariance) {
  // generate particles which distribution is the normal distribution with
  // mean 'old_mean' and covariance 'old_covariance'.
  // noises are also added

  CovarianceMatrix Cholesky_L(
      old_covariance.llt().matrixL()); // old_covariance == Cholesky_L *
                                       // Cholesky_L.transpose()

  for (int i = 0; i < number_of_particles; i++) {
    // The return values of get_UND_particle() follows multivariate normal
    // distribution with mean: 0 and covariance: the identity matrix In general,
    // when x follows multivariate normal distribution with mean m and
    // covariance C, Ax + b follows multivariate normal distribution with mean
    // Am + b and covariance A * C * A^T So the following value follows the
    // wanted normal distribution
    particles[i] = old_mean + Cholesky_L * get_UND_particle();
    // Add noise
    particles[i] += noise_variance.cwiseProduct(get_UND_particle());
  }
}

void PoseEstimator::calculate_touch_likelihoods(
    const unsigned char &touched_object_id,
    const object_geometry_ptr &gripped_geometry,
    const fcl::Transform3f &gripper_transform) {
  for (int i = 0; i < number_of_particles; i++) {
    double distance = calculate_distance(
        touched_objects[touched_object_id], gripped_geometry,
        gripper_transform * particle_to_transform(particles[i]));
    likelihoods[i] = (std::abs(distance) < distance_threshold ? 1.0 : 0.0);
  }
}

void PoseEstimator::calculate_new_distribution(
    Particle &new_mean, CovarianceMatrix &new_covariance) {
  // calculate mean and covariance of particles with the weight likelihoods
  // divided by its sum

  double sum_of_likelihoods =
      std::accumulate(likelihoods.begin(), likelihoods.end(), 0.0);
  std::cout << "The sum of likelihoods:" << sum_of_likelihoods << '\n';
  if (sum_of_likelihoods <= 0.0) {
    throw std::runtime_error("The sum of likelihoods is 0");
  }
  new_mean.setZero();
  new_covariance.setZero();
  // Note that Cov[X, Y] = E[XY] - E[X]E[Y]
  for (int i = 0; i < number_of_particles; i++) {
    new_mean += likelihoods[i] / sum_of_likelihoods * particles[i];
    new_covariance += likelihoods[i] / sum_of_likelihoods * particles[i] *
                      particles[i].transpose();
  }
  new_covariance -= new_mean * new_mean.transpose();
}

void make_BVHModel(object_geometry_ptr &bvhmodel,
                   const std::vector<Eigen::Vector3d> &vertices,
                   const std::vector<boost::array<int, 3>> &triangles) {
  bvhmodel = object_geometry_ptr(new object_geometry());
  std::vector<fcl::Vec3f> fcl_points(vertices.size());
  for (int i = 0; i < vertices.size(); i++) {
    fcl_points[i] = fcl::Vec3f(vertices[i][0], vertices[i][1], vertices[i][2]);
  }
  std::vector<fcl::Triangle> fcl_triangles(triangles.size());
  for (int j = 0; j < triangles.size(); j++) {
    fcl_triangles[j] =
        fcl::Triangle(triangles[j][0], triangles[j][1], triangles[j][2]);
  }
  bvhmodel->beginModel(fcl_points.size(), fcl_triangles.size());
  bvhmodel->addSubModel(fcl_points, fcl_triangles);
  bvhmodel->endModel();
}

void PoseEstimator::touched_step(
    const unsigned char &touched_object_id,
    const std::vector<Eigen::Vector3d> &vertices,
    const std::vector<boost::array<int, 3>> &triangles,
    const fcl::Transform3f &gripper_transform, const Particle &old_mean,
    const CovarianceMatrix &old_covariance, Particle &new_mean,
    CovarianceMatrix &new_covariance) {
  generate_particles(old_mean, old_covariance);
  object_geometry_ptr gripped_geometry;
  make_BVHModel(gripped_geometry, vertices, triangles);
  calculate_touch_likelihoods(touched_object_id, gripped_geometry,
                              gripper_transform);
  calculate_new_distribution(new_mean, new_covariance);
}

void PoseEstimator::place_step(
    const std::vector<Eigen::Vector3d> &vertices,
    const std::vector<boost::array<int, 3>> &triangles,
    const Eigen::Transform<double, 3, Eigen::Isometry> &gripper_transform,
    const double &support_surface, const Particle &old_mean,
    const CovarianceMatrix &old_covariance, Particle &new_mean,
    CovarianceMatrix &new_covariance) {
  // load geometry and calculate the center of gravity
  int number_of_vertices = vertices.size();
  Eigen::Vector3d center_of_gravity_of_gripped =
      calculate_center_of_gravity(vertices, triangles);
  // calculate the coordinates of vertices of the object when the pose is the
  // given mean
  auto mean_transform = particle_to_eigen_transform(old_mean);
  auto current_center_of_gravity =
      gripper_transform * mean_transform * center_of_gravity_of_gripped;
  std::vector<Eigen::Vector3d> current_vertices(number_of_vertices);
  for (int i = 0; i < number_of_vertices; i++) {
    current_vertices[i] = gripper_transform * mean_transform * vertices[i];
  }

  // calculate the three vertices of the object touching the ground
  int ground_touch_vertex_id_1, ground_touch_vertex_id_2,
      ground_touch_vertix_id_3; // The first point touching the ground, and
                                // the second and the third.
  Eigen::Quaterniond rotation;
  bool stability;
  find_three_points(current_vertices, current_center_of_gravity,
                    ground_touch_vertex_id_1, ground_touch_vertex_id_2,
                    ground_touch_vertix_id_3, rotation, stability);

  // If the object is not stable after placing, throw exception
  if (!stability) {
    throw std::runtime_error("Unstable after placing");
  }

  // Update the distribution
  place_update_distribution(
      old_mean, old_covariance, center_of_gravity_of_gripped,
      vertices[ground_touch_vertex_id_1], vertices[ground_touch_vertex_id_2],
      vertices[ground_touch_vertix_id_3], support_surface, gripper_transform,
      new_mean, new_covariance);
}

cv::Point3d to_cv_point(const Eigen::Vector3d &p) {
  return cv::Point3d(p(0), p(1), p(2));
}

cv::Mat PoseEstimator::generate_image(
    const std::vector<Eigen::Vector3d> &vertices,
    const std::vector<boost::array<int, 3>> &triangles,
    const Eigen::Transform<double, 3, Eigen::Isometry> &transform,
    const cv::Size &image_size) {
  // generated the estimated binary image seen from the camera when the pose
  // of the gripped object in the world is represented by 'transform'

  // Transform the coordinates of the gripped object and convert to
  // cv::Point3d
  int number_of_vertices = vertices.size();
  std::vector<cv::Point3d> object_points(number_of_vertices);
  for (int i = 0; i < number_of_vertices; i++) {
    auto current_ver = transform * vertices[i];
    object_points[i] = to_cv_point(current_ver);
  }

  // Calculate the points projected to the image
  std::vector<cv::Point2d> image_points;
  cv::projectPoints(object_points, camera_r, camera_t, camera_matrix,
                    camera_dist_coeffs, image_points);

  // Fill all triangles of the gripped objects on the image
  cv::Mat image = cv::Mat::zeros(image_size, CV_8UC1);
  for (int j = 0; j < triangles.size(); j++) {
    cv::Point triangle[3];
    for (int k = 0; k < 3; k++) {
      triangle[k] = image_points[triangles[j][k]];
    }
    cv::fillConvexPoly(image, &triangle[0], 3, 1);
  }
  return image;
}

double PoseEstimator::similarity_of_images(const cv::Mat &estimated_image,
                                           const cv::Mat &binary_looked_image,
                                           const cv::Mat &ROI_mask) {
  // Calculate the similarity of two binary images
  // The similarity is defined as the number pixels on which both images are 1
  // divided by the number of pixels on which at least one image is 1 in the
  // range of interest By definition, the range of similarity is [0.0, 1.0]

  cv::Mat union_image, intersection_image;
  cv::bitwise_or(estimated_image, binary_looked_image, union_image, ROI_mask);
  cv::bitwise_and(estimated_image, binary_looked_image, intersection_image,
                  ROI_mask);
  int union_sum = cv::countNonZero(union_image);
  int intersection_sum = cv::countNonZero(intersection_image);
  return union_sum > 0
             ? (double)intersection_sum / (double)union_sum
             : 1.0; // If both images are all 0, similarity is defined as 1.0
}

void PoseEstimator::calculate_look_likelihoods(
    const std::vector<Eigen::Vector3d> &vertices,
    const std::vector<boost::array<int, 3>> &triangles,
    const Eigen::Transform<double, 3, Eigen::Isometry> &gripper_transform,
    const cv::Mat &binary_looked_image,
    const boost::array<unsigned int, 4> &range_of_interest) {
  cv::Mat ROI_mask = cv::Mat::zeros(binary_looked_image.size(), CV_8UC1);
  cv::Point ROI_rectangle[4];
  ROI_rectangle[0] = cv::Point(range_of_interest[0], range_of_interest[2]);
  ROI_rectangle[1] = cv::Point(range_of_interest[1], range_of_interest[2]);
  ROI_rectangle[2] = cv::Point(range_of_interest[1], range_of_interest[3]);
  ROI_rectangle[3] = cv::Point(range_of_interest[0], range_of_interest[3]);
  cv::fillConvexPoly(ROI_mask, ROI_rectangle, 4, 1);
  for (int i = 0; i < number_of_particles; i++) {
    auto estimated_image = generate_image(
        vertices, triangles,
        gripper_transform * particle_to_eigen_transform(particles[i]),
        binary_looked_image.size());
    likelihoods[i] =
        similarity_of_images(estimated_image, binary_looked_image, ROI_mask);
  }
}

void PoseEstimator::to_binary_image(const cv::Mat &bgr_image,
                                    cv::Mat &binary_image) {
  // convert 'bgr_image', a BGR8 image, to the binary image 'binary_image'

  // First, convert it to grayscale image
  cv::Mat gray_image;
  cv::cvtColor(bgr_image, gray_image, cv::COLOR_BGR2GRAY);
  // Apply thresholding
  cv::threshold(gray_image, binary_image, look_threshold, 1,
                cv::THRESH_BINARY_INV);
}

void PoseEstimator::look_step(
    const std::vector<Eigen::Vector3d> &vertices,
    const std::vector<boost::array<int, 3>> &triangles,
    const Eigen::Transform<double, 3, Eigen::Isometry> &gripper_transform,
    const cv::Mat &looked_image,
    const boost::array<unsigned int, 4> &range_of_interest,
    const Particle &old_mean, const CovarianceMatrix &old_covariance,
    Particle &new_mean, CovarianceMatrix &new_covariance) {
  cv::Mat binary_looked_image;
  to_binary_image(looked_image, binary_looked_image);
  generate_particles(old_mean, old_covariance);
  calculate_look_likelihoods(vertices, triangles, gripper_transform,
                             binary_looked_image, range_of_interest);
  calculate_new_distribution(new_mean, new_covariance);
}
