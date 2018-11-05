#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <alproxies/almotionproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>
#include "SBModule/include/SBModule.h"
#include "Utils/include/ConfigManager.h"
#include "TNRSBase/include/BaseIncludes.h"

using namespace std;

typedef boost::shared_ptr<AL::ALMotionProxy> ALMotionProxyPtr;
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
	//! Path to robot's configuration files
  ConfigManager::setDirPaths("Robots/Sim/");
  SharedMemoryPtr sharedMemory;
  sharedMemory = boost::make_shared<SharedMemory>();
  sharedMemory->init();
  cout << "Memory initiated successfully." << endl;
  //ALMemoryProxyPtr memoryProxy = boost::make_shared<AL::ALMemoryProxy>(argv[1], 9559);
  //ALDCMProxyPtr dcmProxy = boost::make_shared<AL::DCMProxy>(argv[1], 9559);
  ALMotionProxyPtr motionProxy =
    boost::make_shared < AL::ALMotionProxy > (argv[1], 9559);
  int* dummyParent;
  auto sbModule = boost::make_shared < SBModule > (dummyParent, motionProxy);
  sbModule->setLocalSharedMemory(sharedMemory);
  sbModule->setupModule();
  sbModule->startModule();
  while(true);
  return 1;
}
