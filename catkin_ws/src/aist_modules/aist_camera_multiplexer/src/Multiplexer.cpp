/*!
 *  \file	Multiplexer.cpp
 *  \author	Toshio UESHIBA
 *  \brief	ROS node for applying filters to depth images
 */
#include "Multiplexer.h"

namespace aist_camera_multiplexer
{
/************************************************************************
*  class Multiplexer::Subscribers					*
************************************************************************/
Multiplexer::Subscribers::Subscribers(Multiplexer* multiplexer,
				      int camera_number)
    :_camera_info_sub(multiplexer->_nh.subscribe<camera_info_t>(
			  "/camera_info" + std::to_string(camera_number), 1,
			  boost::bind(&Multiplexer::camera_info_cb,
				      multiplexer, _1, camera_number))),
     _image_sub( multiplexer->_nh.subscribe<image_t>(
		     "/image" + std::to_string(camera_number), 1,
		     boost::bind(&Multiplexer::image_cb,
				 multiplexer, _1, camera_number))),
     _depth_sub( multiplexer->_nh.subscribe<image_t>(
		     "/depth" + std::to_string(camera_number), 1,
		     boost::bind(&Multiplexer::depth_cb,
				 multiplexer, _1, camera_number))),
     _normal_sub(multiplexer->_nh.subscribe<image_t>(
		     "/normal" + std::to_string(camera_number), 1,
		     boost::bind(&Multiplexer::normal_cb,
				 multiplexer, _1, camera_number)))
{
}

/************************************************************************
*  class Multiplexer							*
************************************************************************/
Multiplexer::Multiplexer(const ros::NodeHandle& nh)
    :_nh(nh),
     _subscribers(),
     _camera_number(0),
     _ddr(_nh),
     _it(_nh),
     _image_pub( _it.advertise("image",  1)),
     _depth_pub( _it.advertise("depth",  1)),
     _normal_pub(_it.advertise("normal", 1)),
     _camera_info_pub(_nh.advertise<camera_info_t>("camera_info", 1))
{
    std::vector<std::string>	camera_names;
    if (!_nh.getParam("camera_names", camera_names))
    {
	ROS_ERROR_STREAM("(Multiplexer) no camera names specified.");
	return;
    }

    std::map<std::string, int>	enum_cameras;
    for (int camera_number = 0; camera_number < camera_names.size();
	 ++camera_number)
    {
	_subscribers.emplace_back(new Subscribers(this, camera_number));
	enum_cameras[camera_names[camera_number]] = camera_number;
    }

    _ddr.registerEnumVariable<int>("active_camera", 0,
				   boost::bind(&Multiplexer::activate_camera,
					       this, _1),
				   "Currently active camera", enum_cameras);
    _ddr.publishServicesTopics();
}

void
Multiplexer::run()
{
    ros::spin();
}

void
Multiplexer::activate_camera(int camera_number)
{
    if (0 <= camera_number && camera_number < _subscribers.size())
    {
	_camera_number = camera_number;

	ROS_INFO_STREAM("(Multiplexer) activate camera number["
			<< _camera_number << ']');
    }
    else
    {
	ROS_ERROR_STREAM("(Multiplexer) requested camera number["
			 << camera_number << "] is out of range");
    }
}

void
Multiplexer::camera_info_cb(const camera_info_cp& camera_info,
			    int camera_number) const
{
    if (camera_number == _camera_number)
	_camera_info_pub.publish(camera_info);
}

void
Multiplexer::image_cb(const image_cp& image, int camera_number) const
{
    if (camera_number == _camera_number)
	_image_pub.publish(image);
}

void
Multiplexer::depth_cb(const image_cp& depth, int camera_number) const
{
    if (camera_number == _camera_number)
	_depth_pub.publish(depth);
}

void
Multiplexer::normal_cb(const image_cp& normal, int camera_number) const
{
    if (camera_number == _camera_number)
	_normal_pub.publish(normal);
}

}	// namespace aist_camera_multiplexer
