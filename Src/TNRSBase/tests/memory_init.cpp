#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "../include/SharedMemory.h"
#include "ConfigManager/include/ConfigManager.h"

using namespace std;

int
main()
{
  ConfigManager::setDirPaths("Robots/Sim/");
  SharedMemoryPtr sharedMemory;
  sharedMemory = boost::make_shared<SharedMemory>();
  sharedMemory->init();
  cout << "Memory initiated successfully." << endl;
  string out;
  sharedMemory->getString(out);
  cout << out << endl;
  return 1;
}
