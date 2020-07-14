#include <cstdlib>
#include "HandeyeCalibration.h"

namespace TU
{
template <class T> void
doJob(bool single, bool eye_on_hand)
{
    size_t	nposes;
    std::cin >> nposes;
    std::vector<Transform<T> >	cMo(nposes), wMe(nposes);
    for (size_t n = 0; n < nposes; ++n)
    {
	std::cin >> cMo[n] >> wMe[n];
	std::cout << "=== cMo[" << n << "] ===" << std::endl;
	cMo[n].print(std::cout);

	std::cout << "=== wMe[" << n << "] ===" << std::endl;
	if (eye_on_hand)
	    wMe[n].print(std::cout) << std::endl;
	else
	    wMe[n].inverse().print(std::cout) << std::endl;
    }


    const auto	eMc = (single ? cameraToEffectorSingle(cMo, wMe)
			      : cameraToEffectorDual(cMo, wMe));
    const auto	wMo = objectToWorld(cMo, wMe, eMc);
    evaluateAccuracy(std::cout, cMo, wMe, eMc, wMo);
}

}	// namespace TU

int
main(int argc, char* argv[])
{
    bool	single = false, eye_on_hand = false;
    for (int c; (c = getopt(argc, argv, "se")) !=EOF; )
	switch (c)
	{
	  case 's':
	    single = true;
	    break;
	  case 'e':
	    eye_on_hand = true;
	    break;
	}

    TU::doJob<double>(single, eye_on_hand);

    return 0;
}
