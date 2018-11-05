#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>
#include "CommModule/include/CommModule.h"
#include "Utils/include/ConfigManager.h"

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
  int* abc;
  auto comm = boost::make_shared <CommModule> (abc);
  comm->setLocalSharedMemory(sharedMemory);
  comm->setupModule();
  comm->startModule();
  while(true);
  return 1;
}
