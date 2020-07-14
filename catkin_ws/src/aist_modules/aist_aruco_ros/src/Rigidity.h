/*!
* \file		Rigidity.h
* \author	Toshio UESHIBA
* \brief	Class definition represending rigid transformations
*/
#include <opencv2/core/core.hpp>
#include "opencv_aux.h"

namespace aist_aruco_ros
{
/************************************************************************
*  class Rigidity<T, D>							*
************************************************************************/
template <class T, size_t D>
class Rigidity
{
  public:
    using element_t	= T;
    using vector_t	= cv::Matx<T, D, 1>;
    using matrix_t	= cv::Matx<T, D, D>;

  public:
		Rigidity(const vector_t& t=vector_t::zeros(),
			 const matrix_t& R=matrix_t::eye())
		    :_t(t), _R(R)			{}
    template <class ITER>
		Rigidity(ITER begin, ITER end)		{ fit(begin, end); }

    const auto&	t()				const	{ return _t; }
    const auto&	R()				const	{ return _R; }
    vector_t	operator ()(const vector_t& x)	const	{ return _t + _R * x; }
    template <class ITER>
    element_t	fit(ITER begin, ITER end)		;

  private:
    vector_t	_t;
    matrix_t	_R;
};

template <class T, size_t D> template <class ITER>
typename Rigidity<T, D>::element_t
Rigidity<T, D>::fit(ITER begin, ITER end)
{
  // 充分な個数の点対があるか？
    const size_t	ndata = std::distance(begin, end);
    if (ndata < D)		// beginが有効か？
	throw std::invalid_argument("Rigidity::fit(): not enough data!!");

  // 重心の計算
    vector_t	xc(vector_t::zeros()), yc(vector_t::zeros());
    for (auto corres = begin; corres != end; ++corres)
    {
	xc += corres->first;
	yc += corres->second;
    }
    const auto	k = element_t(1)/element_t(ndata);
    xc *= k;
    yc *= k;

  // モーメント行列の計算
    element_t	sqr_d = 0;
    matrix_t	M(matrix_t::zeros());
    for (auto corres = begin; corres != end; ++corres)
    {
	const vector_t	dx = corres->first  - xc;
	const vector_t	dy = corres->second - yc;

	sqr_d += (dx.dot(dx) + dy.dot(dy));
	M     += dx % dy;
    }

  // モーメント行列を特異値分解し，U, Vの行列式が正になるように補正
    cv::SVD	svd(M, cv::SVD::FULL_UV);
    if (cv::determinant(svd.u) < 0)
    {
	svd.u.col(D-1) *= -1;
	svd.w.at<element_t>(D-1) *= -1;
    }
    if (cv::determinant(svd.vt) < 0)
    {
	svd.vt.row(D-1) *= -1;
	svd.w.at<element_t>(D-1) *= -1;
    }

  // 点群間の相似変換の計算
    cv::transpose(svd.u * svd.vt, _R);
    _t = yc - _R * xc;

  // 残差平均の計算
    for (size_t i = 0; i < D; ++i)
	sqr_d -= 2 * svd.w.at<element_t>(i);

    return std::sqrt(std::abs(sqr_d)/ndata);
}

template <class T, size_t D> std::ostream&
operator <<(std::ostream& out, const Rigidity<T, D>& similarity)
{
    return out << similarity.R() << std::endl
	       << similarity.t() << std::endl;
}

}	// namespace aist_aruco_ros
