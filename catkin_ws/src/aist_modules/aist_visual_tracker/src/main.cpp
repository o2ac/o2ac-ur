/*!
* \file		main.cpp
* \author	Toshio UESHIBA
*/
#include <ros/ros.h>
#include "KLTTracker.h"

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "aist_visual_tracker");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
    // 				   ros::console::levels::Debug);

    try
    {
	ros::NodeHandle			nh("~");
	aist_visual_tracker::KLTTracker	tracker(nh);
	tracker.run();
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what() << std::endl;
	return 1;
    }

    return 0;
}
