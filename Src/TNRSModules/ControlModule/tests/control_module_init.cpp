#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>
#include "ControlModule/include/ControlModule.h"
#include "Utils/include/ConfigManager.h"
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"

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
  ALMemoryProxyPtr memoryProxy =
    boost::make_shared < AL::ALMemoryProxy > (argv[1], 9559);
  ALDCMProxyPtr dcmProxy = boost::make_shared < AL::DCMProxy > (argv[1], 9559);
  int* parent;
  auto controlModule =
    boost::make_shared < ControlModule > (parent, memoryProxy, dcmProxy);
  controlModule->setLocalSharedMemory(sharedMemory);
  controlModule->setupModule();
  controlModule->startModule();
  while(true);
  return 1;
}
