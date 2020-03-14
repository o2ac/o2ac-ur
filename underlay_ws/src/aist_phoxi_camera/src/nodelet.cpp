/*!
 *  \file	nodelet.cpp
 */
#include "Camera.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace aist_phoxi_camera
{
/************************************************************************
*  class CameraNodelet							*
************************************************************************/
class CameraNodelet : public nodelet::Nodelet
{
  public:
    CameraNodelet()							{}

    virtual void	onInit()					;
    void		timer_callback(const ros::TimerEvent&)		;

  private:
    ros::NodeHandle		_nh;
    boost::shared_ptr<Camera>	_node;
    ros::Timer			_timer;
};

void
CameraNodelet::onInit()
{
    NODELET_INFO("aist_phoxi_camera::CameraNodelet::onInit()");

    _nh = getNodeHandle();
    _node.reset(new Camera(getName()));
    std::cout << "rate = " << 1.0/_node->rate() << std::endl;
    _timer = _nh.createTimer(ros::Duration(1.0/_node->rate()),
			     &CameraNodelet::timer_callback, this);

}

void
CameraNodelet::timer_callback(const ros::TimerEvent&)
{
    _node->tick();
}

}

PLUGINLIB_EXPORT_CLASS(aist_phoxi_camera::CameraNodelet, nodelet::Nodelet);
