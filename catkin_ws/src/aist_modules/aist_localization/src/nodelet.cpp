/*!
 *  \file	nodelet.cpp
 *  \author	Toshio UESHIBA
 */
#include "Localization.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace aist_localization
{
/************************************************************************
*  class LocalizationNodelet						*
************************************************************************/
class LocalizationNodelet : public nodelet::Nodelet
{
  public:
			LocalizationNodelet()				{}

    virtual void	onInit()					;

  private:
    ros::NodeHandle			_nh;
    boost::shared_ptr<Localization>	_node;
};

void
LocalizationNodelet::onInit()
{
    NODELET_INFO("aist_localization::LocalizationNodelet::onInit()");
    _node.reset(new Localization(getPrivateNodeHandle()));
}

}	// namespace aist_localization

PLUGINLIB_EXPORT_CLASS(aist_localization::LocalizationNodelet, nodelet::Nodelet);
