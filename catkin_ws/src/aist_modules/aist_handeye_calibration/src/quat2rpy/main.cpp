#include <iostream>
#include <tf/transform_datatypes.h>

namespace TU
{
static tf::Vector3
getRPY(const tf::Quaternion& q)
{
    const tf::Matrix3x3	rot(q);
    tfScalar		roll, pitch, yaw;
    rot.getRPY(roll, pitch, yaw);
    return {roll, pitch, yaw};
}
    
}

int
main()
{
    for (;;)
    {
	constexpr tfScalar	degree = 180.0/M_PI;
	tfScalar		qx, qy, qz, qw;
	char			c;
	
	std::cerr << "> ";
	std::cin >> qx >> c >> qy >> c >> qz >> c >> qw;

	auto	rpy = TU::getRPY(tf::Quaternion(qx, qy, qz, qw));

	std::cout << rpy[0]*degree << ", "
		  << rpy[1]*degree << ", "
		  << rpy[2]*degree << std::endl;
    }
}
