#include <fcl/distance.h>
#include <fcl/math/transform.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/shape/geometric_shapes.h>
#include <Eigen/Geometry>
#include <ccd/vec3.h>
#include <Eigen/Cholesky>

#include <iostream>
#include <random>
#include <stdexcept>

#include "calculation_for_place.cpp"

#include <boost/predef/other/endian.h>

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
  Particle p;
  for(int i = 0; i < 6; i++){
    p(i) = unit_normal_distribution(engine);
  }
  return p;
}

double calculate_distance(const std::shared_ptr<fcl::CollisionObject> &touched_object, const std::shared_ptr<fcl::CollisionGeometry> &gripped_geometry, const fcl::Transform3f &gripped_transform)
{
  std::shared_ptr<fcl::CollisionObject> moved_object(new fcl::CollisionObject(gripped_geometry, gripped_transform));
  fcl::DistanceRequest distance_request;
  fcl::DistanceResult distance_result;
  double distance = fcl::distance(moved_object.get(), touched_object.get(), distance_request, distance_result);
  return distance;
}

Eigen::Vector3d calculate_cog(const std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> &bvhm)
{
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
  int number_of_particles = 0;
  std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> gripped_geometry;
  std::vector<std::shared_ptr<fcl::CollisionObject>> touched_objects;
  double distance_threshold;
  Particle noise_variance;

  std::vector<Particle> particles;
  std::vector<double> likelihoods;

  Eigen::Vector3d cog_of_gripped;
public:

  PoseEstimator(
		// Geometric parameters
		const std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> &gripped_geometry,
		const std::vector<std::shared_ptr<fcl::CollisionObject>> &touched_objects,
		const double &distance_threshold,
		const Particle &noise_variance,
		// parameters for Gaussian particle filter
		const int &number_of_particles
		)
  {
    this->number_of_particles = number_of_particles;
    this->gripped_geometry = gripped_geometry;
    this->touched_objects = touched_objects;
    this->distance_threshold = distance_threshold;
    this->noise_variance = noise_variance;

    particles = std::vector<Particle>(number_of_particles);
    likelihoods = std::vector<double>(number_of_particles);

    cog_of_gripped = calculate_cog(gripped_geometry);
  }

  void generate_particles(const Particle &old_mean, const CovarianceMatrix &old_covariance){
    CovarianceMatrix Cholesky_L(old_covariance.llt().matrixL());
    for(int i=0;i<number_of_particles;i++){
      particles[i] = old_mean + Cholesky_L * get_UND_particle() + noise_variance.cwiseProduct(get_UND_particle());
    }
  }
  
  void calculate_touch_likelihoods(const int &touched_object_id, const fcl::Transform3f &gripper_transform)
  {
    for(int i=0;i<number_of_particles;i++){
      double distance = calculate_distance(touched_objects[touched_object_id], gripped_geometry, particle_to_transform(particles[i]) * gripper_transform);
      likelihoods[i] = (std::abs(distance) < distance_threshold ? 1.0: 0.0);
    }
  }

  void calculate_new_distribution(Particle &new_mean, CovarianceMatrix &new_covariance){
    double sum_of_likelihoods = std::accumulate(likelihoods.begin(), likelihoods.end(), 0.0);
    std::cerr << "living:"<< sum_of_likelihoods << '\n';
    if( sum_of_likelihoods <= 0.0){
      throw std::runtime_error("The sum of likelihoods is 0");
    }
    new_mean.setZero();
    new_covariance.setZero();
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
    int num_of_vertices = gripped_geometry->num_vertices;
    std::vector<Eigen::Vector3d> vertices(num_of_vertices);
    auto cog = gripper_transform * cog_of_gripped;
    for(int i=0; i<num_of_vertices; i++){
      vertices[i] = gripper_transform * to_eigen_vector(gripped_geometry->vertices[i]);
    }
    auto mean_transform = particle_to_eigen_transform(old_mean);
    auto current_cog = mean_transform * cog;
    std::vector<Eigen::Vector3d> current_vertices(num_of_vertices);
    for(int i=0;i<num_of_vertices;i++){
      current_vertices[i] = mean_transform * vertices[i];
    }
    int first_point_id, second_point_id, third_point_id; // The first point touching the ground, and the second and the third.
    Eigen::Quaterniond rotation;
    find_three_points(current_vertices, current_cog, first_point_id, second_point_id, third_point_id, rotation);

    // stability check
    auto rotated_cog = rotation * current_cog;
    std::vector<Eigen::Vector3d> rotated_vertices(num_of_vertices);
    for(int i=0;i<num_of_vertices;i++){
      rotated_vertices[i] = rotation * current_vertices[i];
    }
    bool stability = false;
    const double EPS = 1e-6;
    double min_z = rotated_vertices[first_point_id](2);
    for(int j=0;j<gripped_geometry->num_tris; j++){
      auto ver0 = rotated_vertices[gripped_geometry->tri_indices[j][0]];
      auto ver1 = rotated_vertices[gripped_geometry->tri_indices[j][1]];
      auto ver2 = rotated_vertices[gripped_geometry->tri_indices[j][2]];
      // If the triangle is on the ground and the center is in the above space of it, the object is stable.
      if(ver0(2) <= min_z + EPS
	 && ver1(2) <= min_z + EPS
	 && ver2(2) <= min_z + EPS
	 && ccw(ver0, ver1, rotated_cog) <= EPS
	 && ccw(ver1, ver2, rotated_cog) <= EPS
	 && ccw(ver2, ver0, rotated_cog) <= EPS){
	stability = true;
	break;
      }
    }
    if(!stability){
      throw std::runtime_error("Unstable after placing");
    }
    place_update_distribution(old_mean, old_covariance,
			      cog, vertices[first_point_id], vertices[second_point_id],
			      vertices[third_point_id], ground_z,
			      new_mean, new_covariance);
  }
};
