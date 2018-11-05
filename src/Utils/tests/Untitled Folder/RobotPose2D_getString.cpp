#include <iostream>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "../include/DataUtils.h"
#include "../include/RobotStateDefinitions.h"

using namespace Utils;

int
main()
{
  std::string out;
  RobotPose2D<float> rp = RobotPose2D<float>(0.f, 0.f, 0.f);
  DataUtils::getString<float>(out, rp);
  std::cout << "RobotPose2D string result: " << out << std::endl;
  return 1;
}
