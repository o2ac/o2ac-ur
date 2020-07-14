/*!
 *  \file	DualNumber.h
 *  \author	Toshio UESHIBA
 *  \brief	二重数を表すクラスの定義と実装
 */
#ifndef TU_DUALNUMBER_H
#define TU_DUALNUMBER_H

#include <iostream>
#include <boost/operators.hpp>

namespace TU
{
/************************************************************************
*  class DualNumber<T>							*
************************************************************************/
//! 二重数を表すクラス
/*!
    \param T	要素の型
*/
template <class T>
class DualNumber : boost::field_operators<DualNumber<T> >
{
  public:
    using value_type	= T;	//!< 要素の型
    
  public:
			DualNumber(const value_type& p=value_type(),
				   const value_type& d=value_type())
			    :_p(p), _d(d)			{}
    
    const value_type&	primary()			const	{ return _p; }
    void		primary(const value_type& p)		{ _p = p; }
    const value_type&	dual()				const	{ return _d; }
    void		dual(const value_type& d)		{ _d = d; }

    bool		operator ==(const DualNumber& x)
			{
			    return _p == x._p && _d == x._d;
			}
    
    const DualNumber&	operator +() const
			{
			    return *this;
			}

    DualNumber		operator -() const
			{
			    return {-_p, -_d};
			}

    DualNumber&		operator +=(const DualNumber& x)
			{
			    _p += x._p;
			    _d += x._d;
			    return *this;
			}

    DualNumber&		operator -=(const DualNumber& x)
			{
			    _p -= x._p;
			    _d -= x._d;
			    return *this;
			}

    DualNumber&		operator *=(const value_type& c)
			{
			    _p *= c;
			    _d *= c;
			    return *this;
			}

    DualNumber&		operator /=(const value_type& c)
			{
			    _p /= c;
			    _d /= c;
			    return *this;
			}

    DualNumber&		operator *=(const DualNumber& x)
			{
			    const auto	p = _p;
			    _p *= x._p;
			    (_d *= x._p) += (p * x._d);
			    return *this;
			}

    DualNumber&		operator /=(const DualNumber& x)
			{
			    *this *= inverse(x);
			    return *this;
			}

    friend std::istream&
			operator >>(std::istream& in, DualNumber& x)
			{
			    return in >> x._p >> x._d;
			}

  private:
    value_type	_p;	//!< primary part
    value_type	_d;	//!< dual part
};
    
template <class T> inline DualNumber<T>
square(const DualNumber<T>& x)
{
    return {square(x.primary()), x.primary()*x.dual() + x.dual()*x.primary()};
}
    
template <class T> inline DualNumber<T>
conj(const DualNumber<T>& x)
{
    return {conj(x.primary()), conj(x.dual())};
}
    
template <class T> inline DualNumber<T>
inverse(const DualNumber<T>& x)
{
    const auto	r = inverse(x.primary());
    return {r, -r * x.dual() * r};
}

template <class T> inline std::ostream&
operator <<(std::ostream& out, const DualNumber<T>& x)
{
    return out << '(' << x.primary() << ' ' << x.dual() << ')';
}
    
}
#endif	// !TU_DUALNUMBER_H
