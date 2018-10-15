#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>
#include "LocalizationModule/include/LocalizationModule.h"
#include "Utils/include/ConfigManager.h"
#include "TNRSBase/include/SharedMemory.h"

using namespace std;

typedef boost::shared_ptr<AL::ALMemoryProxy> ALMemoryProxyPtr;
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
  int* dummyParent;
  auto lModule = boost::make_shared < LocalizationModule > (dummyParent);
  lModule->setLocalSharedMemory(sharedMemory);
  lModule->setupModule();
  lModule->startModule();
  auto req1 = boost::make_shared<SwitchLocalization>(true);
  auto req2 = boost::make_shared<SwitchBallObstacle>(true);
  auto req3 = boost::make_shared<ResetLocalizer>();
  BaseModule::publishModuleRequest(req1);
  BaseModule::publishModuleRequest(req2);
  BaseModule::publishModuleRequest(req3);
  while(true);
  return 1;
}
