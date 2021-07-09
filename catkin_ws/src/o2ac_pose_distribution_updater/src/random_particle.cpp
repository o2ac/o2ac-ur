#include "o2ac_pose_distribution_updater/random_particle.hpp"

namespace {
std::random_device seed_generator;
std::default_random_engine engine(seed_generator());
std::normal_distribution<> unit_normal_distribution(0.0, 1.0);
} // namespace

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
