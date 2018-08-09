#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <alproxies/almotionproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include "ConfigManager/include/ConfigManager.h"
#include "TNRSBase/include/SharedMemory.h"
#include "VisionModule/include/VisionModule.h"
#include "VisionModule/include/VisionRequest.h"

using namespace std;

typedef boost::shared_ptr<AL::ALMotionProxy> ALMotionProxyPtr;
typedef boost::shared_ptr<AL::ALMemoryProxy> ALMemoryProxyPtr;
typedef boost::shared_ptr<AL::ALVideoDeviceProxy> ALVideoDeviceProxyPtr;
typedef boost::shared_ptr<AL::DCMProxy> ALDCMProxyPtr;

int
main(int argc, char* argv[])
{
  if (argc != 2) {
    std::cerr << "Wrong number of arguments!" << std::endl;
    std::cerr << "Usage: say NAO_IP" << std::endl;
    exit(2);
  }
  ConfigManager::setDirPaths("Robots/Sim/");
  SharedMemoryPtr sharedMemory;
  sharedMemory = boost::make_shared<SharedMemory>();
  sharedMemory->init();
  cout << "Memory initiated successfully." << endl;
  //ALMemoryProxyPtr memoryProxy = boost::make_shared<AL::ALMemoryProxy>(argv[1], 9559);
  //ALDCMProxyPtr dcmProxy = boost::make_shared<AL::DCMProxy>(argv[1], 9559);
  //ALMotionProxyPtr motionProxy = boost::make_shared<AL::ALMotionProxy>(argv[1], 9559);
  ALVideoDeviceProxyPtr camProxy =
    boost::make_shared < AL::ALVideoDeviceProxy > (argv[1], 9559);
  int* dummyParent;
  auto vModule = boost::make_shared < VisionModule > (dummyParent, camProxy);
  vModule->setLocalSharedMemory(sharedMemory);
  vModule->setupModule();
  vModule->startModule();
  auto vRequest = boost::make_shared<SwitchVision>(true);
  auto fpRequest = boost::make_shared<SwitchFieldProjection>(true);
  auto litRequest = boost::make_shared<SwitchLogImages>(true, 0);
  auto libRequest = boost::make_shared<SwitchLogImages>(true, 1);
  BaseModule::publishModuleRequest(vRequest);
  //BaseModule::publishModuleRequest(litRequest);
  //BaseModule::publishModuleRequest(libRequest);
  //BaseModule::publishModuleRequest(fpRequest);
  while(true);
  return 1;
}
