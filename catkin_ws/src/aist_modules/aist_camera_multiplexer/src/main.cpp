/*!
* \file		main.cpp
* \author	Toshio UESHIBA
*/
#include <ros/ros.h>
#include "Multiplexer.h"

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "aist_camera_multiplexer");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
    // 				   ros::console::levels::Debug);

    try
    {
	ros::NodeHandle				nh("~");
	aist_camera_multiplexer::Multiplexer	multiplexer(nh);
	multiplexer.run();
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what() << std::endl;
	return 1;
    }

    return 0;
}
