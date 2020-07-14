/*!
* \file		opencv_aux.h
* \author	Toshio UESHIBA
* \brief	Some auxiary supporting functions of opencv
*/
#ifndef OPNECV_AUX_H
#define OPNECV_AUX_H
#include <opencv2/core/core.hpp>

namespace aist_aruco_ros
{
/************************************************************************
*  global functions							*
************************************************************************/
template <class T, int M, int N> cv::Matx<T, M, N>
operator %(const cv::Matx<T, M, 1>& x, const cv::Matx<T, N, 1>& y)
{
    cv::Matx<T, M, N>	mat;
    for (size_t i = 0; i < M; ++i)
	for (size_t j = 0; j < N; ++j)
	    mat(i, j) = x(i) * y(j);
    return mat;
}

}
#endif
