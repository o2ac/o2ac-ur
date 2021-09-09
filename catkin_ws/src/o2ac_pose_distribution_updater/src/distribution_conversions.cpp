#include "o2ac_pose_distribution_updater/distribution_conversions.hpp"

class particle_to_perturbation {
public:
  Eigen::Isometry3d mean;

  particle_to_perturbation(const Eigen::Isometry3d &mean) { this->mean = mean; }
  // Neede by Eigen AutoDiff
  enum { InputsAtCompileTime = 6, ValuesAtCompileTime = 6 };

  // Also needed by Eigen AutoDiff
  typedef Eigen::Matrix<double, 6, 1> InputType;
  typedef Eigen::Matrix<double, 6, 1> ValueType;

  template <typename T>
  void operator()(const Eigen::Matrix<T, 6, 1> &particle,
                  Eigen::Matrix<T, 6, 1> *perturbation) const {
    Eigen::Transform<T, 3, Eigen::Isometry> transform =
        particle_to_eigen_transform(particle);
    *perturbation =
        check_operator<T>(-Eigen::Matrix<T, 4, 4>::Identity() +
                          (transform * mean.inverse().cast<T>()).matrix());
  }
};

void eigen_distribution_RPY_to_Lie(const Particle &RPY_mean,
                                   const CovarianceMatrix &RPY_covariance,
                                   Eigen::Isometry3d &Lie_mean,
                                   CovarianceMatrix &Lie_covariance) {
  Lie_mean = particle_to_eigen_transform(RPY_mean);
  Eigen::AutoDiffJacobian<particle_to_perturbation> particle_to_perturbation_AD(
      Lie_mean);
  Eigen::Matrix<double, 6, 1> mean_perturbation;
  CovarianceMatrix Jacobian;
  particle_to_perturbation_AD(RPY_mean, &mean_perturbation, &Jacobian);
  Lie_covariance = Jacobian * RPY_covariance * Jacobian.transpose();
}

class perturbation_to_particle {
public:
  Eigen::Isometry3d mean;

  perturbation_to_particle(const Eigen::Isometry3d &mean) { this->mean = mean; }

  // Neede by Eigen AutoDiff
  enum { InputsAtCompileTime = 6, ValuesAtCompileTime = 6 };

  // Also needed by Eigen AutoDiff
  typedef Eigen::Matrix<double, 6, 1> InputType;
  typedef Eigen::Matrix<double, 6, 1> ValueType;

  template <typename T>
  void operator()(const Eigen::Matrix<T, 6, 1> &perturbation,
                  Eigen::Matrix<T, 6, 1> *particle) const {
    Eigen::Transform<T, 3, Eigen::Isometry> transform =
        Eigen::Transform<T, 3, Eigen::Isometry>(
            Eigen::Matrix<T, 4, 4>::Identity() +
            hat_operator<T>(perturbation)) *
        mean.cast<T>();
    // convert it to Particle
    particle->block(0, 0, 3, 1) = transform.translation();
    Eigen::Quaternion<T> rotation(transform.rotation());
    T roll, pitch, yaw;
    quaternion_to_RPY(rotation.w(), rotation.x(), rotation.y(), rotation.z(),
                      roll, pitch, yaw);
    particle->block(3, 0, 3, 1) << roll, pitch, yaw;
  }
};

void eigen_distribution_Lie_to_RPY(const Eigen::Isometry3d &Lie_mean,
                                   const CovarianceMatrix &Lie_covariance,
                                   Particle &RPY_mean,
                                   CovarianceMatrix &RPY_covariance) {
  Eigen::AutoDiffJacobian<perturbation_to_particle> perturbation_to_particle_AD(
      Lie_mean);
  CovarianceMatrix Jacobian;
  perturbation_to_particle_AD(Particle::Zero(), &RPY_mean, &Jacobian);
  RPY_covariance = Jacobian * Lie_covariance * Jacobian.transpose();
}

void distribution_RPY_to_Lie(
    const geometry_msgs::PoseWithCovariance &RPY_distribution,
    geometry_msgs::PoseWithCovariance &Lie_distribution) {
  Particle RPY_mean = pose_to_particle(RPY_distribution.pose);
  Eigen::Isometry3d Lie_mean;
  CovarianceMatrix RPY_covariance =
                       array_36_to_matrix_6x6(RPY_distribution.covariance),
                   Lie_covariance;
  eigen_distribution_RPY_to_Lie(RPY_mean, RPY_covariance, Lie_mean,
                                Lie_covariance);
  Lie_distribution.pose = RPY_distribution.pose;
  Lie_distribution.covariance = matrix_6x6_to_array_36(Lie_covariance);
}

void distribution_Lie_to_RPY(
    const geometry_msgs::PoseWithCovariance &Lie_distribution,
    geometry_msgs::PoseWithCovariance &RPY_distribution) {
  Eigen::Isometry3d Lie_mean;
  tf::poseMsgToEigen(Lie_distribution.pose, Lie_mean);
  Particle RPY_mean;
  CovarianceMatrix Lie_covariance =
                       array_36_to_matrix_6x6(Lie_distribution.covariance),
                   RPY_covariance;
  eigen_distribution_Lie_to_RPY(Lie_mean, Lie_covariance, RPY_mean,
                                RPY_covariance);
  RPY_distribution.pose = Lie_distribution.pose;
  RPY_distribution.covariance = matrix_6x6_to_array_36(RPY_covariance);
}
