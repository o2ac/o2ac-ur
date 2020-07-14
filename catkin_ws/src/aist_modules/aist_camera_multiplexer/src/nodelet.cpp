/*!
 *  \file	nodelet.cpp
 *  \author	Toshio UESHIBA
 */
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "Multiplexer.h"

namespace aist_camera_multiplexer
{
/************************************************************************
*  class DetectorNodelet						*
************************************************************************/
class MultiplexerNodelet : public nodelet::Nodelet
{
  public:
			MultiplexerNodelet()				{}

    virtual void	onInit()					;

  private:
    boost::shared_ptr<Multiplexer>	_node;
};

void
MultiplexerNodelet::onInit()
{
    NODELET_INFO("aist_camera_multiplexer::MultiplexerNodelet::onInit()");
    _node.reset(new Multiplexer(getPrivateNodeHandle()));
}

}	// namespace aist_camera_multiplexer

PLUGINLIB_EXPORT_CLASS(aist_camera_multiplexer::MultiplexerNodelet,
		       nodelet::Nodelet);
