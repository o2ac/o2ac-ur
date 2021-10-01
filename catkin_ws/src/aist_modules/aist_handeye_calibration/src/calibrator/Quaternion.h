// Software License Agreement (BSD License)
//
// Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of National Institute of Advanced Industrial
//    Science and Technology (AIST) nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Toshio Ueshiba
//
/*!
 *  \file	Quaternion.h
 *  \author	Toshio Ueshiba
 *  \brief	四元数を表すクラスの定義と実装
 */
#ifndef TU_QUATERNION_H
#define TU_QUATERNION_H

#include <boost/operators.hpp>
#include <Eigen/Dense>

namespace TU
{
/************************************************************************
*  global functions							*
************************************************************************/
template <class T> Eigen::Array<T, 1, 3>
rotation_angles(const Eigen::Matrix<T, 3, 3>& R)
{
    if (R(0, 0) == T(0) && R(1, 0) == T(0))
	return {std::atan2(-R(1, 2), R(1, 1)),
		(R(2, 0) < 0 ? M_PI/2 : -M_PI/2),
		T(0)};
    else
	return { std::atan2(R(2, 1), R(2, 2)),
		-std::asin(R(2, 0)),
		 std::atan2(R(1, 0), R(0, 0))};
}

template <class T> Eigen::Matrix<T, 3, 3>
rotation_matrix(const Eigen::Array<T, 3, 1>& rpy)
{
    using matrix33_t   = Eigen::Matrix<T, 3, 3>;
    using angle_axis_t = Eigen::AngleAxis<T>;
    using vector3_t    = Eigen::Matrix<T, 3, 1>;

    matrix33_t	m;
    m = angle_axis_t(rpy(2), vector3_t::UnitZ())
      * angle_axis_t(rpy(1), vector3_t::UnitY())
      * angle_axis_t(rpy(0), vector3_t::UnitX());

    return m;
}

/************************************************************************
*  class Quaternion<T>							*
************************************************************************/
//! 四元数を表すクラス
/*!
    \param T	要素の型
*/
template <class T>
class Quaternion : boost::field_operators<Quaternion<T> >
{
  public:
    using scalar_type	= T;
    using vector_type	= Eigen::Matrix<T, 3, 1>;
    using vector4_type	= Eigen::Matrix<T, 4, 1>;
    using rotation_type	= Eigen::Matrix<T, 3, 3>;
    using array_type	= Eigen::Array<T, 1, 3>;

  public:
			Quaternion(const scalar_type& s=scalar_type(),
				   const scalar_type& x=scalar_type(),
				   const scalar_type& y=scalar_type(),
				   const scalar_type& z=scalar_type())
			    :_s(s), _v({x, y, z})			{}

			Quaternion(const scalar_type& s,
				   const vector_type& v)
			    :_s(s), _v(v)				{}

			Quaternion(const vector4_type& x)
			    :_s(x(0)), _v({x(1), x(2), x(3)})		{}

    const scalar_type&	scalar()			const	{ return _s; }
    void		scalar(const scalar_type& s)		{ _s = s; }
    const vector_type&	vector()			const	{ return _v; }
    void		vector(const vector_type& v)		{ _v = v; }

    Quaternion&		normalize()
			{
			    return *this /= std::sqrt(square(*this));
			}

    bool		operator ==(const Quaternion& x)
			{
			    return _s == x._s && _v(0) == x._v(0)
					      && _v(1) == x._v(1)
					      && _v(2) == x._v(2);
			}

    const Quaternion&	operator +() const
			{
			    return *this;
			}

    Quaternion		operator -() const
			{
			    return {-_s, -_v(0), -_v(1), -_v(2)};
			}

    Quaternion&		operator +=(const Quaternion& x)
			{
			    _s += x._s;
			    _v += x._v;
			    return *this;
			}

    Quaternion&		operator -=(const Quaternion& x)
			{
			    _s -= x._s;
			    _v -= x._v;
			    return *this;
			}

    Quaternion&		operator *=(const scalar_type& c)
			{
			    _s *= c;
			    _v *= c;
			    return *this;
			}

    Quaternion&		operator /=(const scalar_type& c)
			{
			    _s /= c;
			    _v /= c;
			    return *this;
			}

    Quaternion&		operator *=(const Quaternion& x)
			{
			    const auto	s = _s;
			    _s = s * x._s - _v.dot(x._v);
			    _v = s * x._v + _v * x._s + _v.cross(x._v);
			    return *this;
			}

    Quaternion&		operator /=(const Quaternion& x)
			{
			    *this *= inverse(x);
			    return *this;
			}

			operator vector4_type() const
			{
			    vector4_type	x;
			    x << _s, _v(0), _v(1), _v(2);
			    return x;
			}

    rotation_type	R() const
			{
			    vector_type		q = 2 * _v;
			    rotation_type	Q = q * _v.transpose();
			    const auto		c = _s * _s - _v.squaredNorm();
			    Q(0, 0) += c;
			    Q(1, 1) += c;
			    Q(2, 2) += c;

			    q *= _s;
			    Q(0, 1) -= q(2);
			    Q(0, 2) += q(1);
			    Q(1, 0) += q(2);
			    Q(1, 2) -= q(0);
			    Q(2, 0) -= q(1);
			    Q(2, 1) += q(0);

			    return Q;
			}

    rotation_type	Rt() const
			{
			    return conj(*this).R();
			}

    array_type		rpy() const
			{
			    return rotation_angles(R());
			}

    scalar_type		theta() const
			{
			    return 2 * std::acos(_s);
			}

    vector_type		n() const
			{
			    return _v.normalized();
			}

    scalar_type		angular_distance(const Quaternion& q) const
			{
			    const auto	d = *this * conj(q);

			    return 2 * std::atan2(d.vector().norm(),
						  std::abs(d.scalar()));
			}

    std::ostream&	put(std::ostream& out) const
			{
			    return out << _v(0) << ' ' << _v(1) << ' '
				       << _v(2) << ' ' << _s;
			}

    std::istream&	get(std::istream& in)
			{
			    return in >> _v(0) >> _v(1) >> _v(2) >> _s;
			}

    friend std::istream&
			operator >>(std::istream& in, Quaternion& x)
			{
			    return in >> x._s >> x._v(0) >> x._v(1) >> x._v(2);
			}

  private:
    scalar_type	_s;	//!< scalar part
    vector_type	_v;	//!< vector part
};

template <class T> inline T
square(const Quaternion<T>& x)
{
    return x.scalar() * x.scalar() + x.vector().squaredNorm();
}

template <class T> inline Quaternion<T>
conj(const Quaternion<T>& x)
{
    return {x.scalar(), -x.vector()};
}

template <class T> inline Quaternion<T>
inverse(const Quaternion<T>& x)
{
    auto	q = conj(x);
    q /= square(q);
    return q;
}

template <class T> inline std::ostream&
operator <<(std::ostream& out, const Quaternion<T>& x)
{
    return out << x.scalar()    << ' ' << x.vector()(0) << ' '
	       << x.vector()(1) << ' ' << x.vector()(2) << std::endl;
}

}
#endif	// !TU_QUATERNION_H
