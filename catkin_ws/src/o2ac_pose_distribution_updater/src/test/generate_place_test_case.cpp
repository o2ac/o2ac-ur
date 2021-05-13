/*
This program generates test cases for place test

Usage:
The first argument is the name of stl file representing the gripped object
The second argument is the number 'time' of action call

This program writes 'time' blocks.
Eack block represents a call of place action.
 Each block is of the form:

 gripper_pose_particle
 mean
 covariance
 X
 (new_mean)       or  (error_message)
 (new_covariance)

 'gripper_pose_particle' and 'mean' are represented by 6-dimensional vectors
 and 'covariance' is represented by 6x6 matrix.
 'gripper_pose_particle' represents the gripper pose of the place action call as
a particle. These values are randomly generated. If update for the place action
called by these values is not calculated successfully, X=0 and the error message
follows. Otherwise, X=1 and 'new_mean', which is represented as 6-dimensional
vector, and 'new_covariance', which represented as a 6x6 matrix,  represent the
responsed distribution.
 */

#include "o2ac_pose_distribution_updater/estimator.hpp"
#include "o2ac_pose_distribution_updater/read_stl.hpp"

namespace {
const double pi = 3.1416;

std::random_device seed_generator;
std::default_random_engine engine(seed_generator());
std::uniform_real_distribution<double> dist(0, 1.0);

Particle get_random_particle() {
  Particle p;
  p << 0.1 * dist(engine), 0.1 * dist(engine), 0.1 * dist(engine),
      2.0 * pi * dist(engine), 2.0 * pi * dist(engine), 2.0 * pi * dist(engine);
  return p;
}
} // namespace

#include <iostream>

int main(int argc, char **argv) {
  FILE *in = fopen(argv[1], "r");
  assert(in != NULL);

  std::vector<Eigen::Vector3d> vertices;
  std::vector<boost::array<int, 3>> triangles;
  read_stl(in, vertices, triangles);
  fclose(in);

  PoseEstimator estimator;
  int time = atoi(argv[2]);

  Eigen::IOFormat full_format(Eigen::FullPrecision);
  for (int t = 0; t < time; t++) {
    Particle gripper_pose_particle = get_random_particle(),
             mean = get_random_particle();
    auto gripper_transform = particle_to_eigen_transform(gripper_pose_particle);
    std::cout << gripper_pose_particle.transpose().format(full_format)
              << std::endl;
    std::cout << mean.transpose().format(full_format) << std::endl;
    CovarianceMatrix covariance;
    for (int i = 0; i < 6; i++) {
      for (int j = i; j < 6; j++) {
        covariance(i, j) = covariance(j, i) = dist(engine);
      }
    }
    std::cout << covariance.format(full_format) << std::endl;
    double support_surface = -1.0 - dist(engine);
    std::cout << std::setprecision(15) << support_surface << std::endl;
    Particle new_mean;
    CovarianceMatrix new_covariance;
    try {
      estimator.place_step(vertices, triangles, gripper_transform,
                           support_surface, mean, covariance, new_mean,
                           new_covariance);
    } catch (std::runtime_error &e) {
      puts("0");
      std::cout << e.what() << std::endl;
      continue;
    }
    puts("1");
    std::cout << new_mean.transpose().format(full_format) << std::endl;
    std::cout << new_covariance.format(full_format) << std::endl;
  }
  return 0;
}
