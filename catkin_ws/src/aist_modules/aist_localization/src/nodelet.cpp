/*!
 *  \file	nodelet.cpp
 */
#include "Localization.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace aist_localization
{
/************************************************************************
*  class DetectorNodelet						*
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
    _nh = getNodeHandle();
    _node.reset(new Localization(getName()));
}

}	// namespace aist_localization

PLUGINLIB_EXPORT_CLASS(aist_localization::LocalizationNodelet, nodelet::Nodelet);
