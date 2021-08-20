/*!
 *  \file Plane.h
 */
#include <iostream>
#include <opencv2/core.hpp>
#include <ros/ros.h>

namespace TU
{
/************************************************************************
*  struct Plane<T, N>							*
************************************************************************/
template <class T, size_t N>
class Plane
{
  public:
    using vector_type	= cv::Vec<T, N>;
    using value_type	= T;

  public:
    Plane()					    :_n(), _d(0)	{}
    Plane(const vector_type& norm, value_type dist) :_n(norm), _d(dist)	{}
    template <class ITER>
    Plane(ITER begin, ITER end)				{ fit(begin, end); }

    template <class ITER>
    void		fit(ITER begin, ITER end)	;

    constexpr
    static size_t	ndataMin()			{ return N;  }
    const vector_type&	normal()		const	{ return _n; }
    value_type		distance()		const	{ return _d; }
    value_type		sdistance(const vector_type& x) const
			{
			    return _n.dot(x) + _d;
			}
    value_type		distance(const vector_type& x) const
			{
			    return std::abs(sdistance(x));
			}
    vector_type		cross_point(const vector_type& v,
				    const vector_type& x={0, 0, 0}) const
			{
			    return x - (sdistance(x)/_n.dot(v)) * v;
			}
    vector_type		projection(const vector_type& x) const
			{
			    return x - sdistance(x) * _n;
			}

    friend std::ostream&
			operator <<(std::ostream& out, const Plane& plane)
			{
			    out << plane._n << ": " << plane._d;
			}

  private:
    vector_type	_n;	// normal
    value_type	_d;	// distance from the origin
};

template <class T, size_t N> template <class ITER> void
Plane<T, N>::fit(ITER begin, ITER end)
{
    using matrix_type = cv::Matx<T, N, N>;

    const auto	ext   =	[](const auto& x)
			{
			    matrix_type	y;
			    for (size_t i = 0; i < N; ++i)
				for (size_t j = 0; j < N; ++j)
				    y(i, j) = x(i) * x(j);
			    return y;
			};

  // Check #points.
    const auto	npoints = std::distance(begin, end);
    if (npoints < N)
	throw std::runtime_error("Failed to fit a plane: " + std::to_string(N)
				 + " or more points required!");

  // Compute centroid.
    auto	centroid = vector_type::zeros();
    for (auto iter = begin; iter != end; ++iter)
	centroid += *iter;
    centroid *= value_type(1)/value_type(npoints);

  // Compute moment matrix.
    auto	moments = matrix_type::zeros();
    for (auto iter = begin; iter != end; ++iter)
	moments += ext(*iter - centroid);

    matrix_type	evectors;
    vector_type	evalues;
    cv::eigen(moments, evalues, evectors);

    _n = vector_type::all(0);
    for (size_t j = 0; j < N; ++j)
	_n(j) = evectors(N - 1, j);
    _d = -_n.dot(centroid);
    if (_d < 0)
    {
	_n *= value_type(-1);
	_d *= -1;
    }

    ROS_DEBUG_STREAM("plane = " << *this << ", err = "
		     << std::sqrt(std::abs(evalues(N-1))/value_type(npoints))
		     << ", computed from " << npoints << " points.");
}

template <class T, int N> T
angle(const cv::Vec<T, N>& x, const cv::Vec<T, N>& y)
{
    return std::acos(x.dot(y)/(cv::norm(x)*cv::norm(y))) * 180.0/M_PI;
}

template <class T> cv::Matx<T, 3, 3>
rodrigues(const cv::Matx<T, 3, 1>& r)
{
    using matrix_t = cv::Matx<T, 3, 3>;

    const auto	theta = r.norm();
    r /= theta;
    const auto	c = std::cos(theta), s = std::sin(theta);
    matrix_t	R = matrix_t::zeros();
    for (int i = 0; i < 3; ++i)
    {
	R(i, i) += c;

	for (int j = 0; j < 3; ++j)
	    R(i, j) += (1 - c)*r(i)*r(j);
    }
    R(0, 1) -= s * r(2);
    R(0, 2) += s * r(1);
    R(1, 0) += s * r(2);
    R(1, 2) -= s * r(0);
    R(2, 0) -= s * r(1);
    R(2, 1) += s * r(0);

    return R;
}

}	// TU
