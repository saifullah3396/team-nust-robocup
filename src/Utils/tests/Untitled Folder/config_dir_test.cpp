#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "../include/ConfigManager.h"

int
main()
{
  ConfigManager::setDirPaths("Robots/Sim/");
#ifdef MODULE_IS_REMOTE
  std::cout << "REMOTE." << endl;
#else
  std::cout << "LOCAL." << endl;
#endif
  return 1;
}
