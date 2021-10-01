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
  \file		HandeyeCalibration.h
  \brief	Algorithmts for handeye calibration.
  \author	Toshio Ueshiba
*/
#ifndef HANDEYECALIBRATION_H
#define HANDEYECALIBRATION_H

#include <Eigen/Dense>
#include "Transform.h"

namespace TU
{
namespace detail
{
/************************************************************************
*  static functions							*
************************************************************************/
template <class T> static Eigen::Matrix<T, 3, 3>
skew(const Eigen::Matrix<T, 3, 1>& v)
{
    Eigen::Matrix<T, 3, 3>	m;
    m <<     0, -v(2),  v(1),
	  v(2),     0, -v(0),
	 -v(1),  v(0),     0;

    return m;
}

#if 1
template <class T> static Eigen::Matrix<T, 4, 4>
qdiff_matrix(const Quaternion<T>& a, const Quaternion<T>& b)
{
    Eigen::Matrix<T, 4, 4>	C;
    C(0, 0)			 = a.scalar() - b.scalar();
    C.template block<3, 1>(1, 0) = a.vector() - b.vector();
    C(0, 1)			 = -C(1, 0);
    C(0, 2)			 = -C(2, 0);
    C(0, 3)			 = -C(3, 0);
    C.template block<3, 3>(1, 1) = skew((a.vector() + b.vector()).eval());
    C(1, 1)			+= C(0, 0);
    C(2, 2)			+= C(0, 0);
    C(3, 3)			+= C(0, 0);

    return C;
}
#else
template <class T> static Eigen::Matrix<T, 3, 4>
qdiff_matrix(const Quaternion<T>& a, const Quaternion<T>& b)
{
    Eigen::Matrix<T, 3, 4>	C;
    const auto	sdiff		 = a.scalar() - b.scalar();
    const auto	vdiff		 = a.vector() - b.vector();
    C[0, 0)			 = vdiff(0);
    C(1, 0)			 = vdiff(1);
    C(2, 0)			 = vdiff(2);
    C.template block<3, 3>(0, 1) = skew(a.vector() + b.vector());
    C(0, 1)			+= sdiff;
    C(1, 2)			+= sdiff;
    C(2, 3)			+= sdiff;

    return C;
}
#endif

template <class E>
static DualNumber<Quaternion<typename E::value_type> >
computeDualQuaternion(const Eigen::Block<const E, 4, 1>& up,
		      const Eigen::Block<const E, 4, 1>& ud,
		      const Eigen::Block<const E, 4, 1>& vp,
		      const Eigen::Block<const E, 4, 1>& vd)
{
    using quaternion_t	= Quaternion<typename E::value_type>;

    auto	a = up.dot(ud);
    auto	b = up.dot(vd) + vp.dot(ud);
    auto	c = vp.dot(vd);

    if (std::abs(a) > std::abs(c))
    {
	c /= a;
	b /= 2*a;

	const auto	x0 = (b > 0 ? -b - std::sqrt(b*b - c)
				    : -b + std::sqrt(b*b - c));
	const auto	x1 = c/x0;
	const auto	l0 = (x0*up + vp).norm();
	const auto	l1 = (x1*up + vp).norm();

	if (l0 > l1)
	    return {quaternion_t((x0/l0)*up + (1/l0)*vp),
		    quaternion_t((x0/l0)*ud + (1/l0)*vd)};
	else
	    return {quaternion_t((x1/l1)*up + (1/l1)*vp),
		    quaternion_t((x1/l1)*ud + (1/l1)*vd)};
    }
    else
    {
	a /= c;
	b /= 2*c;

	const auto	y0 = (b > 0 ? -b - std::sqrt(b*b - a)
				    : -b + std::sqrt(b*b - a));
	const auto	y1 = a/y0;
	const auto	l0 = (up + y0*vp).norm();
	const auto	l1 = (up + y1*vp).norm();

	if (l0 > l1)
	    return {quaternion_t((1/l0)*up + (y0/l0)*vp),
		    quaternion_t((1/l0)*ud + (y0/l0)*vd)};
	else
	    return {quaternion_t((1/l1)*up + (y1/l1)*vp),
		    quaternion_t((1/l1)*ud + (y1/l1)*vd)};
    }
}

}	// namespace detail
/************************************************************************
*  global functions							*
************************************************************************/
template <class T> Transform<T>
cameraToEffectorSingle(const std::vector<Transform<T> >& cMo,
		       const std::vector<Transform<T> >& wMe)
{
    using vector3_t	= Eigen::Matrix<T, 3, 1>;
    using matrix33_t	= Eigen::Matrix<T, 3, 3>;
    using matrix44_t	= Eigen::Matrix<T, 4, 4>;
    using quaternion_t	= Quaternion<T>;

    if (cMo.size() != wMe.size())
	throw std::runtime_error("transformations with different sizes("
				 + std::to_string(cMo.size()) + "!="
				 + std::to_string(wMe.size()) + ").");
    else if (cMo.size() < 2)
	throw std::runtime_error("too few transformations("
				 + std::to_string(cMo.size()) + ").");

    const auto	nposes = cMo.size();

  // Compute rotation.
    matrix44_t	M(matrix44_t::Zero());	// 4x4 matrix initialized with zeros.
    for (size_t i = 0; i < nposes; ++i)
	for (size_t j = i + 1; j < nposes; ++j)
	{
	    const auto	A = wMe[j].inverse() * wMe[i];
	    const auto	B = cMo[j] * cMo[i].inverse();
	    const auto	C = detail::qdiff_matrix(A.primary(), B.primary());
	    M += C.transpose() * C;
	}
    Eigen::SelfAdjointEigenSolver<matrix44_t>	esolver(M);
    quaternion_t	q(esolver.eigenvectors().col(0));  // computed rotation
#ifdef DEBUG
    std::cerr << "--- M ---\n" << M
	      << "\n--- Eigenvectors ---\n" << esolver.eigenvectors()
	      << "\n--- Eigenvalues ---\n" << esolver.eigenvalues()
	      << std::endl;
#endif

  // Compute translation.
    matrix33_t	N(matrix33_t::Zero());
    vector3_t	t(vector3_t::Zero());
    for (size_t i = 0; i < nposes; ++i)
	for (size_t j = i + 1; j < nposes; ++j)
	{
	    const auto	A  = wMe[j].inverse() * wMe[i];
	    const auto	B  = cMo[j] * cMo[i].inverse();
	    auto	Rt = A.Rt();
	    Rt(0, 0) -= 1;
	    Rt(1, 1) -= 1;
	    Rt(2, 2) -= 1;
	    N += Rt * Rt.transpose();
	    t += Rt * (q.R()*B.t() - A.t());
	}
#ifdef DEBUG
    std::cerr << "--- N ---\n" << N << std::endl;
#endif
    t = N.colPivHouseholderQr().solve(t);

    return {t, q};
}

template <class T> Transform<T>
cameraToEffectorDual(const std::vector<Transform<T> >& cMo,
		     const std::vector<Transform<T> >& wMe)
{
    using vector8_t	= Eigen::Matrix<T, 8, 1>;
    using matrix88_t	= Eigen::Matrix<T, 8, 8>;

    if (cMo.size() != wMe.size())
	throw std::runtime_error("transformations with different sizes("
				 + std::to_string(cMo.size()) + "!="
				 + std::to_string(wMe.size()) + ").");
    else if (cMo.size() < 2)
	throw std::runtime_error("too few transformations("
				 + std::to_string(cMo.size()) + ").");

    const auto	nposes = cMo.size();

  // Compute rotation.
    matrix88_t	M(matrix88_t::Zero());	// 4x4 matrix initialized with zeros.
    for (size_t i = 0; i < nposes; ++i)
	for (size_t j = i + 1; j < nposes; ++j)
	{
	    const auto	A = wMe[j].inverse() * wMe[i];
	    const auto	B = cMo[j] * cMo[i].inverse();
	    const auto	C = detail::qdiff_matrix(A.primary(), B.primary());
	    const auto	D = detail::qdiff_matrix(A.dual(),    B.dual());
	    M.template block<4, 4>(0, 0) += C.transpose() * C
					  + D.transpose() * D;
	    M.template block<4, 4>(4, 0) += C.transpose() * D;
	    M.template block<4, 4>(4, 4) += C.transpose() * C;
	}

    Eigen::SelfAdjointEigenSolver<matrix88_t>	esolver(M);
    const auto					U = esolver.eigenvectors();
#ifdef DEBUG
    std::cerr << "--- M ---\n" << M
	      << "\n--- Eigenvectors ---\n" << U
	      << "\n--- Eigenvalues ---\n" << esolver.eigenvalues()
	      << std::endl;
#endif
    const Transform<T>	eMc(detail::computeDualQuaternion(
				U.template block<4, 1>(0, 1),
				U.template block<4, 1>(4, 1),
				U.template block<4, 1>(0, 0),
				U.template block<4, 1>(4, 0)));

    return eMc;
}

template <class T> Transform<T>
objectToWorld(const std::vector<Transform<T> >& cMo,
	      const std::vector<Transform<T> >& wMe, const Transform<T>& eMc)
{
    using vector4_type	= typename Quaternion<T>::vector4_type;

    const auto		nposes = cMo.size();
    vector4_type	p(vector4_type::Zero()), d(vector4_type::Zero());
    for (size_t i = 0; i < nposes; ++i)
    {
      // Transformation from object to world computed from estimated eMc.
	const auto	M = wMe[i] * eMc * cMo[i];
	p += M.primary().operator vector4_type();
	d += M.dual().operator vector4_type();
    }
    p.normalize();
    d /= nposes;
    d -= p*(p.dot(d));

    return DualNumber<Quaternion<T> >(p, d);
}

template <class T> void
evaluateAccuracy(std::ostream& out,
		 const std::vector<Transform<T> >& cMo,
		 const std::vector<Transform<T> >& wMe,
		 const Transform<T>& eMc, const Transform<T>& wMo)
{
    const auto	nposes = cMo.size();
    T		tdiff_mean = 0;	// mean translational distance
    T		tdiff_max  = 0;	// max. translational distance
    T		adiff_mean = 0;	// mean angular distance
    T		adiff_max  = 0;	// max. angular distance

    for (size_t i = 0; i < nposes; ++i)
    {
	const auto	AX    = wMe[i] * eMc;
	const auto	YBinv = wMo * cMo[i].inverse();
	const auto	tdiff = AX.translational_distance(YBinv);
	const auto	adiff = AX.angular_distance(YBinv);

	if (tdiff > tdiff_max)
	    tdiff_max = tdiff;
	if (adiff > adiff_max)
	    adiff_max = adiff;

	tdiff_mean += tdiff*tdiff;
	adiff_mean += adiff*adiff;
    }
    tdiff_mean = std::sqrt(tdiff_mean/nposes);
    adiff_mean = std::sqrt(adiff_mean/nposes);

    constexpr T	degree = 180.0/M_PI;
    out << "\n=== estimated camera pose ====\n";
    eMc.print(out);
    out << "\n=== estimated marker pose ===\n";
    wMo.print(out);
    out << "\ntrans. err(m)  : (mean, max) = ("
	<< tdiff_mean << ", " << tdiff_max
	<< ")\nangle err(deg.): (mean, max) = ("
	<< adiff_mean * degree << ", " << adiff_max * degree << ')'
	<< std::endl;
}
}	// namespace TU
#endif	// !HANDEYECALIBRATION_H
