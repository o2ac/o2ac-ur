/*
The implementation of estimator, which calculates the updated distribution for each action
 */

#include <fcl/distance.h>
#include <fcl/math/transform.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/shape/geometric_shapes.h>
#include <Eigen/Geometry>
#include <ccd/vec3.h>
#include <Eigen/Cholesky>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <random>
#include <stdexcept>

#include "calculation_for_place.cpp"


//Conversion functions

// Note that Particle is Eigen::Matrix<double, 6, 1> and CovarianceMatrix is Eigen::Matrix<double, 6, 6>

fcl::Transform3f particle_to_transform(const Particle &p)
{
  double w,x,y,z;
  RPY_to_quaternion(p(3), p(4), p(5), w, x, y, z);
  return fcl::Transform3f(fcl::Quaternion3f(w, x, y, z),
			  fcl::Vec3f(p(0), p(1), p(2)));
}

Eigen::Transform<double, 3, Eigen::Isometry> particle_to_eigen_transform(const Particle &p)
{
  return Eigen::Translation3d(p.block(0,0,3,1))
    * Eigen::AngleAxis<double>(p(5), Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxis<double>(p(4), Eigen::Vector3d::UnitY())
    * Eigen::AngleAxis<double>(p(3), Eigen::Vector3d::UnitX());
}

Eigen::Vector3d to_eigen_vector(fcl::Vec3f v)
{
  Eigen::Vector3d ev;
  ev << v.data[0], v.data[1], v.data[2];
  return ev;
}

Eigen::Transform<double, 3, Eigen::Isometry> to_eigen_transform(fcl::Transform3f t)
{
  return Eigen::Translation3d(to_eigen_vector(t.getTranslation()))
    * Eigen::Quaterniond(t.getQuatRotation().getW(),
			 t.getQuatRotation().getX(),
			 t.getQuatRotation().getY(),
			 t.getQuatRotation().getZ());
}


std::random_device seed_generator;
std::default_random_engine engine(seed_generator());
std::normal_distribution<> unit_normal_distribution(0.0, 1.0);

Particle get_UND_particle()
{
  //Random Particle generator
  
  // All coordinates are independent
  // The distribution of each coordinate is the normal distribution with mean 0.0, variance 1.0

  Particle p;
  for(int i = 0; i < 6; i++){
    p(i) = unit_normal_distribution(engine);
  }
  return p;
}

double calculate_distance(const std::shared_ptr<fcl::CollisionObject> &touched_object, const std::shared_ptr<fcl::CollisionGeometry> &gripped_geometry, const fcl::Transform3f &gripped_transform)
{
  //Calculate the distance between 'touched_object' and 'gripped_geometry' transfromed by 'gripped_transform'
  
  std::shared_ptr<fcl::CollisionObject> moved_object(new fcl::CollisionObject(gripped_geometry, gripped_transform));
  fcl::DistanceRequest distance_request;
  fcl::DistanceResult distance_result;
  double distance = fcl::distance(moved_object.get(), touched_object.get(), distance_request, distance_result);
  return distance;
}

Eigen::Vector3d calculate_cog(const std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> &bvhm)
{
  // Calculate the center of gravity
  
  std::vector<Eigen::Vector3d> vertices(bvhm->num_vertices);
  for(int i=0;i<bvhm->num_vertices; i++){
    vertices[i] = to_eigen_vector(bvhm->vertices[i]);
  }
  Eigen::Vector3d cog;
  cog.setZero();
  double total_volume = 0.0;
  for(int j=0; j<bvhm->num_tris; j++){
    auto ver0 = vertices[bvhm->tri_indices[j][0]];
    auto ver1 = vertices[bvhm->tri_indices[j][1]];
    auto ver2 = vertices[bvhm->tri_indices[j][2]];
    double tetra_volume = ver0.dot(ver1.cross(ver2));
    cog += tetra_volume * (ver0 + ver1 + ver2);
    total_volume += tetra_volume;
  }
  cog /= 4.0 * total_volume;
  return cog;
}

class PoseEstimator
{
private:
  // Information about gripped object
  std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> gripped_geometry;
  int num_of_vertices;
  std::vector<Eigen::Vector3d> vertices; //  coordinates of vertices as Eigen vector
  Eigen::Vector3d cog_of_gripped; // Center of Gravity, calculated for place action

  // Parameters for Gaussian particle filter
  int number_of_particles;
  Particle noise_variance;
  // Variables for Gaussian particle filter
  std::vector<Particle> particles;
  std::vector<double> likelihoods;

  // Parameters for touch action
  std::vector<std::shared_ptr<fcl::CollisionObject>> touched_objects;
  double distance_threshold;

  // Parameters for look action
  unsigned char look_threshold;
  std::vector<cv::Point3d> calibration_object_points;
  cv::Mat camera_matrix = cv::Mat::eye(3,3, CV_64FC1), camera_dist_coeffs;
  // Variables for look action
  cv::Mat camera_r, camera_t;

public:

  PoseEstimator(const std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> &gripped_geometry)
  {
    this->gripped_geometry = gripped_geometry;

    num_of_vertices = gripped_geometry->num_vertices;
    vertices = std::vector<Eigen::Vector3d>(num_of_vertices);
    for(int i=0; i<num_of_vertices; i++){
      vertices[i] = to_eigen_vector(gripped_geometry->vertices[i]);
    }
    cog_of_gripped = calculate_cog(gripped_geometry);
  }
  
  void set_particle_parameters(const int &number_of_particles,
			       const Particle &noise_variance)
  {
    this->number_of_particles = number_of_particles;
    this->noise_variance = noise_variance;

    particles = std::vector<Particle>(number_of_particles);
    likelihoods = std::vector<double>(number_of_particles);
  }
  
  void set_touch_parameters(const std::vector<std::shared_ptr<fcl::CollisionObject>> &touched_objects,
			    const double &distance_threshold)
  {
    this->touched_objects = touched_objects;
    this->distance_threshold = distance_threshold;
  }

  void set_look_parameters(const double &look_threshold,
			   const std::vector<std::vector<double> > calibration_points,
			   const double &camera_fx,
			   const double &camera_fy,
			   const double &camera_cx,
			   const double &camera_cy)
  {
    this->look_threshold = look_threshold;
    calibration_object_points=std::vector<cv::Point3d>(calibration_points.size());
    for(int i=0;i<calibration_points.size();i++){
      auto p=calibration_points[i];
      calibration_object_points[i]=cv::Point3d(p[0], p[1], p[2]);
    }
    camera_matrix.at<double>(0,0) = camera_fx;
    camera_matrix.at<double>(1,1) = camera_fy;
    camera_matrix.at<double>(0,2) = camera_cx;
    camera_matrix.at<double>(1,2) = camera_cy;
  }

  void generate_particles(const Particle &old_mean, const CovarianceMatrix &old_covariance){
    // generated particles which distribution is the normal distribution with mean 'old_mean' and covariance 'old_covariance' and add noise

    CovarianceMatrix Cholesky_L(old_covariance.llt().matrixL()); // old_covariance == Cholesky_L * Cholesky_L.transpose()

    for(int i=0;i<number_of_particles;i++){
      particles[i] = old_mean + Cholesky_L * get_UND_particle(); // normal distribution with given parameters
      particles[i] += noise_variance.cwiseProduct(get_UND_particle()); // add noise
    }
  }
  
  void calculate_touch_likelihoods(const int &touched_object_id, const fcl::Transform3f &gripper_transform)
  {
    for(int i=0;i<number_of_particles;i++){
      double distance = calculate_distance(touched_objects[touched_object_id], gripped_geometry, gripper_transform * particle_to_transform(particles[i]));
      likelihoods[i] = (std::abs(distance) < distance_threshold ? 1.0: 0.0);
    }
  }

  void calculate_new_distribution(Particle &new_mean, CovarianceMatrix &new_covariance){
    // calculate mean and covariance of particles with the weight likelihoods divided by its sum
    
    double sum_of_likelihoods = std::accumulate(likelihoods.begin(), likelihoods.end(), 0.0);
    std::cout << "The sum of likelihoods:"<< sum_of_likelihoods << '\n';
    if( sum_of_likelihoods <= 0.0){
      throw std::runtime_error("The sum of likelihoods is 0");
    }
    new_mean.setZero();
    new_covariance.setZero();
    // Note that Cov[X, Y] = E[XY] - E[X]E[Y]
    for(int i=0;i<number_of_particles;i++){
      new_mean += likelihoods[i]/sum_of_likelihoods * particles[i];
      new_covariance += likelihoods[i]/sum_of_likelihoods * particles[i] * particles[i].transpose();
    }
    new_covariance -= new_mean * new_mean.transpose();
  }

  void touched_step(const int &touched_object_id, const fcl::Transform3f &gripper_transform, const Particle &old_mean, const CovarianceMatrix &old_covariance, Particle &new_mean, CovarianceMatrix &new_covariance)
  {
    generate_particles(old_mean, old_covariance);
    calculate_touch_likelihoods(touched_object_id, gripper_transform);
    calculate_new_distribution(new_mean, new_covariance);
  }

  void place_step(const Eigen::Transform<double, 3, Eigen::Isometry> &gripper_transform, const double &ground_z, const Particle &old_mean, const CovarianceMatrix &old_covariance, Particle &new_mean, CovarianceMatrix &new_covariance)
  {
    // calculate the coordinates of vertices of the object when the pose is the given mean
    auto mean_transform = particle_to_eigen_transform(old_mean);
    auto current_cog = gripper_transform * mean_transform * cog_of_gripped;
    std::vector<Eigen::Vector3d> current_vertices(num_of_vertices);
    for(int i=0;i<num_of_vertices;i++){
      current_vertices[i] = gripper_transform * mean_transform * vertices[i];
    }

    // calculate the three vertices of the object touching the ground
    int first_point_id, second_point_id, third_point_id; // The first point touching the ground, and the second and the third.
    Eigen::Quaterniond rotation;
    bool stability;
    find_three_points(current_vertices, current_cog, first_point_id, second_point_id, third_point_id, rotation, stability);

    // If the object is not stable after placing, throw exception
    if(!stability){
      throw std::runtime_error("Unstable after placing");
    }

    // Update the distribution
    place_update_distribution(old_mean, old_covariance,
			      cog_of_gripped, vertices[first_point_id], vertices[second_point_id],
			      vertices[third_point_id], ground_z, gripper_transform,
			      new_mean, new_covariance);
  }

  void calibration(const std::vector<cv::Point2d> &calibration_image_points)
  {
    // calculate the pose of camera, which is represented by 'camera_r' and 'camera_t', using calibration points
    cv::solvePnP(calibration_object_points, calibration_image_points,
		 camera_matrix, camera_dist_coeffs, camera_r, camera_t);
  }

  cv::Point3d to_cv_point(const Eigen::Vector3d &p)
  {
    return cv::Point3d(p(0), p(1), p(2));
  }
  
  cv::Mat generate_image(const Eigen::Transform<double, 3, Eigen::Isometry> &trans, const int &height, const int &width)
  {
    // generated the estimated binary image seen from the camera when the pose of the gripped object in the world is represented by 'trans'

    // Transform the coordinates of the gripped object and convert to cv::Point3d
    std::vector<cv::Point3d> object_points(vertices.size());
    for(int i=0;i<vertices.size();i++){
      auto current_ver = trans * vertices[i];
      object_points[i] = to_cv_point(current_ver);
    }

    // Calculate the points projected to the image
    std::vector<cv::Point2d> image_points;
    cv::projectPoints(object_points, camera_r, camera_t, camera_matrix, camera_dist_coeffs, image_points);

    // Fill all triangles of the gripped objects on the image
    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC1);
    for(int j=0; j<gripped_geometry->num_tris; j++){
      cv::Point triangle[3];
      for(int k=0; k<3; k++){
	triangle[k] = image_points[gripped_geometry->tri_indices[j][k]];
      }
      cv::fillConvexPoly(image, &triangle[0], 3, 1);
    }
    return image;
  }

  double similarity_of_images(const cv::Mat &estimated_image, const cv::Mat &binary_looked_image, const std::vector<int> &range_of_interest)
  {
    // Calculate the similarity of two binary images
    // The similarity is defined as the number pixels on which both images are 1 divided by the number of pixels on which at least one image is 1 in the range of interest
    // By definition, the range of similarity is [0.0, 1.0]
    
    int union_sum=0, inter_sum=0;
    for(int y=range_of_interest[2];y<range_of_interest[3];y++){
      for(int x=range_of_interest[0]; x<range_of_interest[1];x++){
	union_sum+= estimated_image.at<unsigned char>(y, x) || binary_looked_image.at<unsigned char>(y, x);
	inter_sum+= estimated_image.at<unsigned char>(y, x) && binary_looked_image.at<unsigned char>(y, x);
      }
    }
    return union_sum > 0 ? (double)inter_sum / (double)union_sum : 1.0; // If both images are all 0, similarity is defined as 1.0
  }
  
  void calculate_look_likelihoods(const Eigen::Transform<double, 3, Eigen::Isometry> &gripper_transform, const cv::Mat &binary_looked_image, const std::vector<int> &range_of_interest){
    for(int i=0;i<number_of_particles; i++){
      auto estimated_image = generate_image(gripper_transform * particle_to_eigen_transform(particles[i]), binary_looked_image.size().height, binary_looked_image.size().width);
      likelihoods[i] = similarity_of_images(estimated_image, binary_looked_image, range_of_interest);
    }
  }

  void to_binary_image(const cv::Mat &bgr_image, cv::Mat &binary_image){
    // convert 'bgr_image', a BGR8 image, to the binary image 'binary_image'

    // First, convert it to grayscale image
    cv::Mat gray_image;
    cv::cvtColor(bgr_image, gray_image, cv::COLOR_BGR2GRAY);
    // Apply thresholding
    cv::threshold(gray_image, binary_image, look_threshold, 1, cv::THRESH_BINARY_INV);
  }
  
  void look_step(const Eigen::Transform<double, 3, Eigen::Isometry> &gripper_transform, const cv::Mat &looked_image, const std::vector<cv::Point2d> &calibration_image_points, const std::vector<int> &range_of_interest, const Particle &old_mean, const CovarianceMatrix &old_covariance, Particle &new_mean, CovarianceMatrix &new_covariance)
  {
    calibration(calibration_image_points);
    cv::Mat binary_looked_image;
    to_binary_image(looked_image, binary_looked_image);
    generate_particles(old_mean, old_covariance);
    calculate_look_likelihoods(gripper_transform, binary_looked_image, range_of_interest);
    calculate_new_distribution(new_mean, new_covariance);
  }
};
