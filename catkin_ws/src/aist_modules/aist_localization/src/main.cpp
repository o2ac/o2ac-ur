/*!
* \file		localization.cpp
* \author	Toshio UESHIBA
* \brief	ROS action server of Photoneo Localization SDK
*/
#include <ros/ros.h>
#include "Localization.h"

int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "localization");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
				   ros::console::levels::Info);

    try
    {
	aist_localization::Localization	localization("~");
	localization.run();
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what() << std::endl;
	return 1;
    }

    return 0;
}
