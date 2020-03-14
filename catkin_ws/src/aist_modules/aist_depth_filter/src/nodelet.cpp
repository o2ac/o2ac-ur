/*!
 *  \file	nodelet.cpp
 */
#include "DepthFilter.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

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
    ros::NodeHandle			_nh;
    boost::shared_ptr<DepthFilter>	_node;
};

void
DepthFilterNodelet::onInit()
{
    NODELET_INFO("aist_depth_filter::DepthFilterNodelet::onInit()");
    _nh = getNodeHandle();
    _node.reset(new DepthFilter(getName()));
}

}	// namespace aist_depth_filter

PLUGINLIB_EXPORT_CLASS(aist_depth_filter::DepthFilterNodelet, nodelet::Nodelet);
