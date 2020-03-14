/*!
* \file		depth_filter.cpp
* \author	Toshio UESHIBA
* \brief	ARuCo marker detector using both intensity and depth images
*/
#include <ros/ros.h>
#include "DepthFilter.h"

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_filter");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
				   ros::console::levels::Debug);

    try
    {
	aist_depth_filter::DepthFilter	filter("~");
	filter.run();
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what() << std::endl;
	return 1;
    }

    return 0;
}
