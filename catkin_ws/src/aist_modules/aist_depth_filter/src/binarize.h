/*
 *  平成14-19年（独）産業技術総合研究所 著作権所有
 *
 *  創作者：植芝俊夫
 *
 *  本プログラムは（独）産業技術総合研究所の職員である植芝俊夫が創作し，
 *  （独）産業技術総合研究所が著作権を所有する秘密情報です．著作権所有
 *  者による許可なしに本プログラムを使用，複製，改変，第三者へ開示する
 *  等の行為を禁止します．
 *
 *  このプログラムによって生じるいかなる損害に対しても，著作権所有者お
 *  よび創作者は責任を負いません。
 *
 *  Copyright 2002-2007.
 *  National Institute of Advanced Industrial Science and Technology (AIST)
 *
 *  Creator: Toshio UESHIBA
 *
 *  [AIST Confidential and all rights reserved.]
 *  This program is confidential. Any using, copying, changing or
 *  giving any information concerning with this program to others
 *  without permission by the copyright holder are strictly prohibited.
 *
 *  [No Warranty.]
 *  The copyright holder or the creator are not responsible for any
 *  damages caused by using this program.
 *
 *  $Id: binarize.h,v 1.1 2008-11-07 05:34:21 ueshiba Exp $
 */
/*!
  \file		binarize.h
  \brief	大津の2値化アルゴリズムの実装
*/
#include <algorithm>

namespace TU
{
//! 大津の2値化アルゴリズムにより与えられたデータ列を2値化する．
/*!
  データ列は昇順にソートされ，閾値となるデータすなわち後半部の先頭データが返される．
  \param begin	データ列の先頭を示す反復子
  \param end	データ列の末尾の次を示す反復子
  \return	閾値となるデータを示す反復子
*/
template <class Iterator> Iterator
binarize(Iterator begin, Iterator end)
{
  // 要素数と平均値を計算．
    long	n = 0;
    double	mean = 0;
    for (Iterator iter = begin; iter != end; ++iter)
    {
	++n;
	mean += *iter;
    }
    mean /= n;

  // 昇順にソート．
    std::sort(begin, end);

  // 大津の判別基準により最適なしきい値を決定．
    Iterator	thresh = begin;
    long	nLow = 0;		// しきい値以下の要素数
    double	cumulationLow = 0;	// しきい値以下の累積値
    double	interVarianceMax = 0;	// クラス間分散の最大値
    for (Iterator iter = begin, head = begin; iter != end; ++iter)
    {
	if (*iter != *head)
	{
	    double	interVariance = cumulationLow - nLow * mean;
	    ((interVariance *= interVariance) /= nLow) /= (n - nLow);
	    if (interVariance > interVarianceMax)
	    {
		interVarianceMax = interVariance;
		thresh = iter;
	    }
	    head = iter;
	}

	++nLow;
	cumulationLow += *iter;
    }

    return thresh;
}

}
