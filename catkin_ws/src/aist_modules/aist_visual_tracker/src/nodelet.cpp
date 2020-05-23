/*!
 *  \file	nodelet.cpp
 *  \author	Toshio UESHIBA
 */
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "KLTTracker.h"

namespace aist_visual_tracker
{
/************************************************************************
*  class KLTTrackerNodelet						*
************************************************************************/
class KLTTrackerNodelet : public nodelet::Nodelet
{
  public:
			KLTTrackerNodelet()				{}

    virtual void	onInit()					;

  private:
    boost::shared_ptr<KLTTracker>	_node;
};

void
KLTTrackerNodelet::onInit()
{
    NODELET_INFO("aist_visual_tracker::KLTTrackerNodelet::onInit()");
    _node.reset(new KLTTracker(getPrivateNodeHandle()));
}

}	// namespace aist_depth_filter

PLUGINLIB_EXPORT_CLASS(aist_visual_tracker::KLTTrackerNodelet,
		       nodelet::Nodelet);
