/*!
 *  \file	nodelet.cpp
 *  \author	Toshio UESHIBA
 */
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "DepthFilter.h"

namespace aist_depth_filter
{
/************************************************************************
*  class DetectorNodelet						*
************************************************************************/
class DepthFilterNodelet : public nodelet::Nodelet
{
  public:
			DepthFilterNodelet()				{}

    virtual void	onInit()					;

  private:
    boost::shared_ptr<DepthFilter>	_node;
};

void
DepthFilterNodelet::onInit()
{
    NODELET_INFO("aist_depth_filter::DepthFilterNodelet::onInit()");
    _node.reset(new DepthFilter(getPrivateNodeHandle()));
}

}	// namespace aist_depth_filter

PLUGINLIB_EXPORT_CLASS(aist_depth_filter::DepthFilterNodelet, nodelet::Nodelet);
