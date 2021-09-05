#include "o2ac_pose_distribution_updater/random_particle.hpp"
#include <random>

namespace {
std::random_device seed_generator;
std::default_random_engine engine(0);
std::normal_distribution<> unit_normal_distribution(0.0, 1.0);
} // namespace

template <int D> Eigen::Matrix<double, D, 1> get_UND_Vector() {
  // Random Vector generator

  // All coordinates are independent
  // The distribution of each coordinate is the normal distribution with mean
  // 0.0, variance 1.0

  Eigen::Matrix<double, D, 1> p;
  for (int i = 0; i < D; i++) {
    p(i) = unit_normal_distribution(engine);
  }
  return p;
}

Particle get_UND_particle() {
  // Random Particle generator
  return get_UND_Vector<6>();
}

Eigen::Vector3d get_UND_Vector3d() {
  // Random Particle generator
  return get_UND_Vector<3>();
}

std::vector<int> get_random_array(int length, int range) {
  std::vector<int> return_array;
  if (length >= range) {
    return_array.resize(range);
    std::iota(return_array.begin(), return_array.end(), 0);
  } else {
    return_array.resize(length);
    std::uniform_int_distribution<> random_int_generator(0, range - length);
    for (int i = 0; i < length; i++) {
      return_array[i] = random_int_generator(engine);
    }
    std::sort(return_array.begin(), return_array.end());
    for (int i = 1; i < length; i++) {
      return_array[i] += i;
    }
  }
  return return_array;
}
