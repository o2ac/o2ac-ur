/*!
* \file		main.cpp
* \author	Toshio UESHIBA
*/
#include <ros/ros.h>
#include "MotionDetector.h"

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "aist_motion_detector");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
    // 				   ros::console::levels::Debug);

    try
    {
	ros::NodeHandle				nh("~");
	aist_motion_detector::MotionDetector	detector(nh);
	detector.run();
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what() << std::endl;
	return 1;
    }

    return 0;
}
