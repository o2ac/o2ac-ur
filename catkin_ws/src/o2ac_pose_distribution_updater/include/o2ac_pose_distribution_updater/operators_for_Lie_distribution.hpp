#ifndef O2AC_POSE_DISTRIBUTION_UPDATER_OPERATORS_FOR_LIE_DISTRIBUTION_HEADER
#define O2AC_POSE_DISTRIBUTION_UPDATER_OPERATORS_FOR_LIE_DISTRIBUTION_HEADER

#include <unsupported/Eigen/MatrixFunctions>

// The bijection from R^3 to so(3), the Lie algebra corresponding SO(3)
template <typename T>
Eigen::Matrix<T, 3, 3> SO3_hat_operator(const Eigen::Matrix<T, 3, 1> &v) {
  Eigen::Matrix<T, 3, 3> m;
  m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return m;
}

// The bijection from R^6 to se(3), the Lie algebra corresponding SE(3)
template <typename T>
Eigen::Matrix<T, 4, 4> hat_operator(const Eigen::Matrix<T, 6, 1> &v) {
  Eigen::Matrix<T, 4, 4> m;
  m.block(0, 0, 3, 3) = SO3_hat_operator<T>(v.block(3, 0, 3, 1));
  m.block(0, 3, 3, 1) = v.block(0, 0, 3, 1);
  m.block(3, 0, 1, 4).setZero();
  return m;
}

// The inverse function of hat_operator
template <typename T>
Eigen::Matrix<T, 6, 1> check_operator(const Eigen::Matrix<T, 4, 4> &m) {
  Eigen::Matrix<T, 6, 1> v;
  v << m(0, 3), m(1, 3), m(2, 3), m(2, 1), m(0, 2), m(1, 0);
  return v;
}

// The adjoint action on se(3) regarded as the linear endomorphism of R^6
template <typename T>
Eigen::Matrix<T, 6, 6> adjoint(const Eigen::Matrix<T, 6, 1> &v) {
  // return a matrix m such that
  // hat_operator(m * u) = ad_(hat_operator(v)) hat_operator(u) for all u in R^6
  Eigen::Matrix<T, 6, 6> m;
  m.block(0, 0, 3, 3) = m.block(3, 3, 3, 3) =
      SO3_hat_operator<T>(v.block(3, 0, 3, 1));
  m.block(0, 3, 3, 3) = SO3_hat_operator<T>(v.block(0, 0, 3, 1));
  m.block(3, 0, 3, 3).setZero();
  return m;
}

#include <boost/math/special_functions/bernoulli.hpp>
#include <boost/math/special_functions/factorials.hpp>

const int MAX_N = 20;

// the function which sends X to the sum of B_n/n! * X^n where B_n is the n-th
// Bernoulli number
template <typename T> T Bernoulli_series(const T &m) {
  T sum = T::Identity() - m / 2.0, power = m * m; // B_0 = 1, B_1 = -1/2
  // Note that B_n = 0 for odd number n except 1
  for (int n = 2; n < MAX_N; n += 2) {
    sum += boost::math::bernoulli_b2n<double>(n / 2) /
           boost::math::factorial<double>(n) * power;
    power *= m * m;
  }
  return sum;
}

#endif
