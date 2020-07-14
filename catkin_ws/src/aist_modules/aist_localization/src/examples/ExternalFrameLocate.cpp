#include <PhoLocalization.h>
#include <iostream>
#include <memory>

#include <PhoXi.h>

using namespace pho::sdk;
using namespace std;

// example of localization on frame obtained from external device
int
main()
{
    std::unique_ptr<PhoLocalization> Localization;
    try
    {
	Localization.reset(new PhoLocalization());
    }
    catch (const AuthenticationException &ex)
    {
	std::cout << ex.what() << std::endl;
	return -1;
    }

    pho::api::PhoXiFactory	Factory;
    if (!Factory.isPhoXiControlRunning())
    {
	std::cout << "PhoXi Control Software is not running!" << std::endl;

	return 0;
    }

  // here put your device hardware identification
    std::string		HardwareIdentification = "InstalledExamples-basic-example";
    pho::api::PPhoXi	Scanner = Factory.CreateAndConnect(
				    HardwareIdentification,
				    pho::api::PhoXiTimeout::Value::ZeroTimeout);
    if (!Scanner)
    {
	std::cout << "Connection to the device " << HardwareIdentification
		  << " was Unsuccessful!" << std::endl;
	return 0;
    }

    int			frameId = Scanner->TriggerFrame();
    pho::api::PFrame	frame	= Scanner->GetSpecificFrame(frameId);

    SceneSource		Scene;
    try
    {
        Scene = SceneSource::PhoXi(Scanner);
    }
    catch (const pho::sdk::PhoLocalizationException &ex)
    {
        std::cout << "SceneSource Error: " << ex.what() << std::endl;
        return -1;
    }
    Localization->SetSceneSource(Scene);

    Localization->LoadLocalizationConfiguration("T-fitting.plcf");

    AsynchroneResultQueue	AsyncQueue = Localization->StartAsync(frame);
    TransformationMatrix4x4	Transform;

    while (AsyncQueue.GetNext(Transform))
    {
	cout << Transform << endl;
    }

    Localization->StopAsync();

    return 0;
}
