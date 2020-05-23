/*!
 *  \file	nodelet.cpp
 *  \author	Toshio UESHIBA
 */
#include "Detector.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace aist_aruco_ros
{
/************************************************************************
*  class DetectorNodelet						*
************************************************************************/
class DetectorNodelet : public nodelet::Nodelet
{
  public:
			DetectorNodelet()				{}

    virtual void	onInit()					;

  private:
    boost::shared_ptr<Detector>	_node;
};

void
DetectorNodelet::onInit()
{
    NODELET_INFO("aist_aruco_ros::DetectorNodelet::onInit()");
    _node.reset(new Detector(getPrivateNodeHandle()));
}

}	// namespace aist_aruco_ros

PLUGINLIB_EXPORT_CLASS(aist_aruco_ros::DetectorNodelet, nodelet::Nodelet);
