#include <iostream>
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>

int main()
{
  try
  {
    AL::ALMotionProxy motion("127.0.0.1", 9559);
    motion.setStiffnesses("Body", 1.f);
  }
  catch (const AL::ALError& e)
  {
    std::cerr << "Caught exception: " << e.what() << std::endl;
    exit(1);
  }
  exit(0);
}
