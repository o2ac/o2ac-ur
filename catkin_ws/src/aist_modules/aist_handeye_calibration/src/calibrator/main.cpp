/*!
  \file		calibrator_main.cpp
  \brief	Entry point of calibrator node
*/
#include "Calibrator.h"
#include "ros/ros.h"

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "calibrator");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
				   ros::console::levels::Debug);

    try
    {
	ros::NodeHandle				nh("~");
	aist_handeye_calibration::Calibrator	calibrator(nh);
	calibrator.run();
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what() << std::endl;
	return 1;
    }

    return 0;
}
