#include <iostream>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "../include/DataUtils.h"
#include "../include/PositionInput.h"

using namespace Utils;

int
main()
{
  std::string out;
  PositionInput<float> rp = PositionInput<float>(0.f, 0.f, 0.f);
  std::cout << "RobotPose2D string result: " << out << std::endl;
  return 1;
}
