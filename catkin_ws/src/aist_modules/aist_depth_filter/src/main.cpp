/*!
* \file		main.cpp
* \author	Toshio UESHIBA
*/
#include <ros/ros.h>
#include "DepthFilter.h"

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_filter");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
    // 				   ros::console::levels::Debug);

    try
    {
	ros::NodeHandle			nh("~");
	aist_depth_filter::DepthFilter	filter(nh);
	filter.run();
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what() << std::endl;
	return 1;
    }

    return 0;
}
