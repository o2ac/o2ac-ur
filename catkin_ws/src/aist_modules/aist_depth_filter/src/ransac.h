/*!
  \file		Ransac.h
  \author	Toshio UESHIBA
  \brief	RANSACを行う関数の定義と実装
*/
#ifndef TU_RANSAC_H
#define TU_RANSAC_H

#include <iterator>
#include <type_traits>
#include <random>
#include <vector>
#include <stdexcept>

namespace std
{
#if __cplusplus < 201700L
namespace detail
{
  template <class IN, class OUT, class SIZE, class GEN> OUT
  sample(IN in, IN ie, input_iterator_tag, OUT out, random_access_iterator_tag,
	 SIZE n, GEN&& gen)
  {
      using distrib_type = std::uniform_int_distribution<SIZE>;
      using param_type   = typename distrib_type::param_type;

      distrib_type	distrib{};
      SIZE		nsampled = 0;

    // 最初のn個をコピー
      for (; in != ie && nsampled != n; ++in, ++nsampled)
	  out[nsampled] = *in;

      for (auto ninputs = nsampled; in != ie; ++in, ++ninputs)
      {
	  const auto	i = distrib(gen, param_type{0, ninputs});
	  if (i < n)
	      out[i] = *in;
      }

      return out + nsampled;
  }

  template<class IN, class OUT, class CAT, class SIZE, class GEN> OUT
  sample(IN in, IN ie, forward_iterator_tag, OUT out, CAT, SIZE n, GEN&& gen)
  {
      using distrib_type = std::uniform_int_distribution<SIZE>;
      using param_type	 = typename distrib_type::param_type;

      distrib_type	distrib{};
      SIZE		nunsampled = std::distance(in, ie);

      if (n > nunsampled)
	  n = nunsampled;

      for (; n != 0; ++in)
	  if (distrib(gen, param_type{0, --nunsampled}) < n)
	  {
	      *out++ = *in;
	      --n;
	  }

      return out;
  }
}	// namespace detail

/// Take a random sample from a population.
template<class IN, class OUT, class SIZE, class GEN> OUT
sample(IN in, IN ie, OUT out, SIZE n, GEN&& gen)
{
    using in_cat  = typename std::iterator_traits<IN>::iterator_category;
    using out_cat = typename std::iterator_traits<OUT>::iterator_category;

    return detail::sample(in, ie, in_cat{}, out, out_cat{},
			  n, std::forward<GEN>(gen));
}
#endif
}	// namespace std

namespace TU
{
/************************************************************************
*  function ransac							*
************************************************************************/
namespace detail
{
  template <class IN>
  class DefaultSampler
  {
    public:
      DefaultSampler(IN begin, IN end)	:_begin(begin), _end(end)	{}

      IN	begin()	const	{ return _begin; }
      IN	end()	const	{ return _end; }
      size_t	size()	const	{ return std::distance(_begin, _end); }
      template <class OUT_, class GEN_>
      void	operator ()(OUT_ out, size_t npoints, GEN_&& gen) const
		{
		    std::sample(_begin, _end, out,
				npoints, std::forward<GEN_>(gen));
		}

    private:
      const IN	_begin;
      const IN	_end;
  };
}

//! RANSACによってoutlierを含む点集合にモデルを当てはめる．
/*!
  テンプレートパラメータSAMPLERは点集合を表すクラスであり，以下の条件を
  満たすこと：
  -# メンバ関数
	begin() const;	点集合の先頭を指す反復子を返す
	end()   const;	点集合の末尾の次を指す反復子を返す
	size()  const;	点集合のサイズを返す
      を持つ．
  -# テンプレートメンバ関数
	template <class OUT_, class GEN_>
	void operator ()(OUT_ out, size_t npoints, GEN_&& gen) const
      によって，乱数発生器 gen を用いて npoints 個の点をサンプルし，out に
      出力できる．
  テンプレートパラメータMODELは当てはめるべきモデルを表すクラスであり，
  以下の条件を満たすこと：
  -# メンバ関数
	template <class ITER>
	void	Model::fit(ITER first, ITER last);
     によって点集合にモデルを当てはめることができる．
  -# 1.に必要な最少点数をメンバ関数
	size_t	Model::ndataMin() const;
     によって知ることができる．

  テンプレートパラメータCONFORMは点のモデルへの適合性を判定する関数
  オブジェクトであり，
	bool	CONFORM::operator()(const PointSet::Container::value_type& p,
				    const Model& model);
  なるインタフェースによってpがmodelに適合しているか判定できること．

  \param ib		inlierとoutlierを含む点集合の先頭
  \param ie		inlierとoutlierを含む点集合の末尾の次
  \param model		pointSetに含まれるinlierを当てはめるモデル
  \param conform	点のモデルへの適合性を判定する関数オブジェクト
  \param inlierRate	[ib, ie)に含まれる点のうちinlierの割合.
			0 < inlierRate < 1でなければならない
  \param hitRate	RANSACによって正しくinlierを引き当てる確率.
			0 <= hitRate < 1でなければならない
  \return		pointSetに含まれるinlier
*/
template <class SAMPLER, class MODEL, class CONFORM, class T> auto
ransac(const SAMPLER& sampler,
       MODEL& model, CONFORM&& conform, T inlierRate, T hitRate=0.99)
    -> std::vector<std::decay_t<decltype(*sampler.begin())> >
{
    using point_type	= std::decay_t<decltype(*sampler.begin())>;
    using pointset_type	= std::vector<point_type>;

    if (sampler.size() < model.ndataMin())
	throw std::runtime_error(
		"ransac(): not enough points in the given point set!!");

    if (inlierRate >= 1)	// sampler の点がが全てinlierなら...
    {				// モデルを当てはめる.
	model.fit(sampler.begin(), sampler.end());
	return std::move(pointset_type(sampler.begin(), sampler.end()));
    }

    if (inlierRate <= 0)
	throw std::invalid_argument(
		"ransac(): given inline rate is not within (0, 1]!!");
    if (hitRate < 0 || hitRate >= 1)
	throw std::invalid_argument(
		"ransac(): given hit rate is not within [0, 1)!!");

  // 与えられたhitRate，PointSetに含まれるinlierの割合およびModelの生成に
  // 要する最少点数から，サンプリングの必要回数を求める．
    T		tmp = 1;
    for (auto n = model.ndataMin(); n-- > 0; )
	tmp *= inlierRate;
    const auto	ntrials = size_t(std::ceil(std::log(1 - hitRate) /
					   std::log(1 - tmp)));

  // 試行（最小個数の点をサンプル，モデル生成，inlier検出）をntrials回行う．
    pointset_type	maximalSet;
    std::mt19937	gen{std::random_device{}()};
    for (size_t n = 0; n < ntrials; ++n)
    {
	try
	{
	  // 点集合からモデルの計算に必要な最少個数の点をサンプルする．
	    pointset_type	minimalSet;
	    sampler(std::back_inserter(minimalSet), model.ndataMin(), gen);

	  // サンプルした点にモデルを当てはめる．
	    model.fit(minimalSet.begin(), minimalSet.end());
	}
	catch (const std::runtime_error& err)	// 当てはめに失敗したら
	{					// (ex. 特異な点配置など)
	    continue;				// 再度サンプルする.
	}

      // 全点の中で生成されたモデルに適合するもの(inlier)を集める．
	pointset_type	inliers;
	std::copy_if(sampler.begin(), sampler.end(),
		     std::back_inserter(inliers),
		     [&](const point_type& p){ return conform(p, model); });

      // これまで生成されたどのモデルよりもinlierの数が多ければ，それを記録する．
      // なお，サンプルされた点（minimalSetの点）が持つ自由度がモデルの自由度
      // よりも大きい場合は，これらに誤差0でモデルを当てはめられるとは限らない
      // ので，minimalSetの点が全てinliersに含まれる保証はない．よって，
      // inliersのサイズがモデル計算に必要な最少点数以上であることもチェックする．
	if (inliers.size() >  maximalSet.size() &&
	    inliers.size() >= model.ndataMin())
	    maximalSet = std::move(inliers);
    }

  // maximalSetに含まれる点を真のinlierとし，それら全てからモデルを生成する．
    model.fit(maximalSet.begin(), maximalSet.end());

    return maximalSet;
}

//! RANSACによってoutlierを含む点集合にモデルを当てはめる．
/*!
  テンプレートパラメータINは点を指すforward_iteratorである．
  テンプレートパラメータMODELは当てはめるべきモデルを表すクラスであり，
  以下の条件を満たすこと：
  -# メンバ関数
	template <class ITER>
	void	Model::fit(ITER first, ITER last);
     によって点集合にモデルを当てはめることができる．
  -# 1.に必要な最少点数をメンバ関数
	size_t	Model::ndataMin() const;
     によって知ることができる．

  テンプレートパラメータCONFORMは点のモデルへの適合性を判定する関数
  オブジェクトであり，
	bool	CONFORM::operator()(const PointSet::Container::valu_type& p,
				    const Model& model);
  なるインタフェースによってpがmodelに適合しているか判定できること．

  \param ib		inlierとoutlierを含む点集合の先頭
  \param ie		inlierとoutlierを含む点集合の末尾の次
  \param model		pointSetに含まれるinlierを当てはめるモデル
  \param conform	点のモデルへの適合性を判定する関数オブジェクト
  \param inlierRate	[ib, ie)に含まれる点のうちinlierの割合.
			0 < inlierRate < 1でなければならない
  \param hitRate	RANSACによって正しくinlierを引き当てる確率.
			0 <= hitRate < 1でなければならない
  \return		pointSetに含まれるinlier
*/
template <class IN, class MODEL, class CONFORM, class T> inline auto
ransac(IN ib, IN ie,
       MODEL& model, CONFORM&& conform, T inlierRate, T hitRate=0.99)
{
    return ransac(detail::DefaultSampler<IN>(ib, ie),
		  model, conform, inlierRate, hitRate);
}

}	// namespace TU
#endif	// !TU_RANSAC_H
