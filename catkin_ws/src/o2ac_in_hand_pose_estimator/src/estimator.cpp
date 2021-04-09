#include <fcl/distance.h>
#include <fcl/math/transform.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/shape/geometric_shapes.h>
#include <Eigen/Geometry>
#include <ccd/vec3.h>
#include <Eigen/Cholesky>

#include <iostream>
#include <random>
using Real = fcl::FCL_REAL;

using Particle = Eigen::Matrix<Real, 6, 1>;
using CovarianceMatrix  = Eigen::Matrix<Real, 6, 6>;

void RPY_to_quaternion(const Real &roll, const Real &pitch, const Real &yaw, Real &w, Real &x, Real &y, Real &z){
  Real sr=std::sin(roll / 2.0);
  Real cr=std::cos(roll / 2.0);
  Real sp=std::sin(pitch / 2.0);
  Real cp=std::cos(pitch / 2.0);
  Real sy=std::sin(yaw / 2.0);
  Real cy=std::cos(yaw / 2.0);
  w = cr * cp * cy + sr * sp * sy;
  x = sr * cp * cy - cr * sp * sy;
  y = cr * sp * cy + sr * cp * sy;
  z = cr * cp * sy - sr * sp * cy;
}

void quaternion_to_RPY(const Real &w, const Real &x, const Real &y, const Real &z, Real &roll, Real &pitch, Real &yaw){
  roll = std::atan2(2.0*(w*x+y*z), w*w-x*x-y*y+z*z);
  pitch = std::asin(2.0*(w*y-x*z));
  yaw = std::atan2(2.0*(w*z+x*y), w*w+x*x-y*y-z*z);
}

fcl::Transform3f particle_to_transform(const Particle &p)
{
  Real w,x,y,z;
  RPY_to_quaternion(p(3), p(4), p(5), w, x, y, z);
  return fcl::Transform3f(fcl::Quaternion3f(w, x, y, z),
			  fcl::Vec3f(p(0), p(1), p(2)));
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

Real calculate_distance(const std::shared_ptr<fcl::CollisionObject> &touched_object, const std::shared_ptr<fcl::CollisionGeometry> &gripped_geometry, const fcl::Transform3f &gripped_transform)
{
  std::shared_ptr<fcl::CollisionObject> moved_object(new fcl::CollisionObject(gripped_geometry, gripped_transform));
  fcl::DistanceRequest distance_request;
  fcl::DistanceResult distance_result;
  Real distance = fcl::distance(moved_object.get(), touched_object.get(), distance_request, distance_result);
  return distance;
}

class PoseEstimator
{
private:
  int number_of_particles = 0;
  std::shared_ptr<fcl::CollisionGeometry> gripped_geometry;
  std::vector<std::shared_ptr<fcl::CollisionObject>> touched_objects;
  Real distance_threshold;
  Particle noise_variance;

  std::vector<Particle> particles;
  std::vector<Real> likelihoods;

public:

  PoseEstimator(
		// Geometric parameters
		const std::shared_ptr<fcl::CollisionGeometry> &gripped_geometry,
		const std::vector<std::shared_ptr<fcl::CollisionObject>> &touched_objects,
		const Real &distance_threshold,
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
    likelihoods = std::vector<Real>(number_of_particles);
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
      Real distance = calculate_distance(touched_objects[touched_object_id], gripped_geometry, particle_to_transform(particles[i]) * gripper_transform);
      likelihoods[i] = (std::abs(distance) < distance_threshold ? 1.0: 0.0);
    }
  }

  int calculate_new_distribution(Particle &new_mean, CovarianceMatrix &new_covariance){
    Real sum_of_likelihoods = std::accumulate(likelihoods.begin(), likelihoods.end(), 0.0);
    std::cerr << "living:"<< sum_of_likelihoods << '\n';
    if( sum_of_likelihoods <= 0.0){
      return 1;
    }    
    new_mean.setZero();
    new_covariance.setZero();
    for(int i=0;i<number_of_particles;i++){
      new_mean += likelihoods[i]/sum_of_likelihoods * particles[i];
      new_covariance += likelihoods[i]/sum_of_likelihoods * particles[i] * particles[i].transpose();
    }
    new_covariance -= new_mean * new_mean.transpose();
    return 0;
  }

  int touched_step(const int &touched_object_id, const fcl::Transform3f &gripper_transform, const Particle &old_mean, const CovarianceMatrix &old_covariance, Particle &new_mean, CovarianceMatrix &new_covariance)
  {
    generate_particles(old_mean, old_covariance);
    calculate_touch_likelihoods(touched_object_id, gripper_transform);
    return calculate_new_distribution(new_mean, new_covariance);
  }
};
