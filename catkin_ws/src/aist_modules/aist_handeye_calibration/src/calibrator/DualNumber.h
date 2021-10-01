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
 *  \file	DualNumber.h
 *  \author	Toshio Ueshiba
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
