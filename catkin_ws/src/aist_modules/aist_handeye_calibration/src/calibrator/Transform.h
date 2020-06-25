/*!
 *  \file	Transform.h
 *  \author	Toshio UESHIBA
 *  \brief	3D rigid transformation represented by dual quaternion
 */
#ifndef TU_TRANSFORM_H
#define TU_TRANSFORM_H

#include <geometry_msgs/Transform.h>
#include "DualNumber.h"
#include "Quaternion.h"

namespace TU
{
/************************************************************************
*  class Transform<T>							*
************************************************************************/
template <class T>
class Transform : boost::multipliable<Transform<T> >
{
  public:
    using quaternion_type	= Quaternion<T>;
    using dual_quaternion_type	= DualNumber<quaternion_type>;
    using scalar_type		= T;
    using vector_type		= typename quaternion_type::vector_type;
    using translation_type	= typename quaternion_type::vector_type;
    using rotation_type		= typename quaternion_type::rotation_type;
    using array_type		= typename quaternion_type::array_type;

  public:
			Transform(const dual_quaternion_type& x)
			    :_dq(x)
			{
			    if (primary().scalar() < 0)
				_dq *= -1;
#ifdef DEBUG
			    const Eigen::Matrix<T, 4, 1>	p(primary()),
								d(dual());
			    std::cerr << "p*p-1 = " << square(p) - 1
			    	      << std::endl;
			    std::cerr << "p*d   = " << p.dot(d) << std::endl;
#endif
			}

			Transform(const translation_type&
				      t=translation_type::Zero(),
				  const quaternion_type&
				      q=quaternion_type(1))
			    :Transform(dual_quaternion_type(
					   q, 0.5*quaternion_type(0, t)*q))
			{
			}

			Transform(const geometry_msgs::Transform& trans)
			    :Transform({trans.translation.x,
					trans.translation.y,
					trans.translation.z},
				       {trans.rotation.w,
					trans.rotation.x,
					trans.rotation.y,
					trans.rotation.z})
			{
			}

			operator geometry_msgs::Transform() const
			{
			    geometry_msgs::Transform	ret;
			    const auto			translation = t();
			    ret.translation.x = translation(0);
			    ret.translation.y = translation(1);
			    ret.translation.z = translation(2);
			    ret.rotation.w    = primary().scalar();
			    ret.rotation.x    = primary().vector()(0);
			    ret.rotation.y    = primary().vector()(1);
			    ret.rotation.z    = primary().vector()(2);

			    return ret;
			}

    Transform&		operator *=(const Transform& x)
			{
			    _dq *= x._dq;
			    if (primary().scalar() < 0)
				_dq *= -1;
			    return *this;
			}

    const auto&		dq()	  const	{ return _dq; }
    const auto&		primary() const	{ return dq().primary(); }
    const auto&		dual()	  const	{ return dq().dual(); }
    Transform		inverse() const	{ return conj(dq()); }
    rotation_type	R()	  const	{ return primary().R(); }
    rotation_type	Rt()	  const	{ return R().transpose(); }
    array_type		rpy()	  const	{ return primary().rpy(); }
    scalar_type		theta() const
			{
			    return 2 * std::acos(primary().scalar());
			}
    vector_type		n()	  const	{ return primary().vec().normalized();}
    scalar_type		d()	  const	{ return n().dot(t()); }

    translation_type	t() const
			{
			    return (2 * dual() * conj(primary())).vector();
			}

    auto		translational_distance(const Transform& trns) const
			{
			    return (t() - trns.t()).norm();
			}

    auto		angular_distance(const Transform& trns) const
			{
			    return primary().angular_distance(trns.primary());
			}

    std::ostream&	print(std::ostream& out) const
			{
			    constexpr scalar_type	degree = 180.0/M_PI;

			    return out << "xyz(m)    = " << t()(0)
				       << ' ' << t()(1) << ' ' << t()(2)
				       << "\nrot(deg.) = " << rpy()*degree
				       << std::endl;
			}

  private:
    dual_quaternion_type	_dq;
};

template <class T> inline std::istream&
operator >>(std::istream& in, Transform<T>& x)
{
    typename Transform<T>::translation_type	t;
    char					c;
    in >> t(0) >> t(1) >> t(2) >> c;

    typename Transform<T>::quaternion_type	q;
    q.get(in);				// Get vector part first, then scalar.

    x = Transform<T>(t, q);

    return in;
}


template <class T> inline std::ostream&
operator <<(std::ostream& out, const Transform<T>& x)
{
    const auto	tt = x.t();
    out << tt(0) << ' ' << tt(1) << ' ' << tt(2) << "; ";
    return x.primary().put(out);	// Put vector part first, then scalar.
}

}	// namespace TU
#endif	// !TU_TRANSFORM_H
