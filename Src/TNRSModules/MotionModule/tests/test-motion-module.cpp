/**
 * @file MotionModule/test-motion-module.cpp
 *
 * This main file loads the ModuleModule tests in naoqi for usage.
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 01 Feb 2017
 */

#include <string>
#include "Utils/include/ConfigManager.h"
#include "MotionModule/tests/include/MotionModuleTests.h"

#ifndef _WIN32
#include <signal.h>
#endif

#ifdef MODULE_IS_REMOTE
#define ALCALL
#else
#ifdef _WIN32
#define ALCALL __declspec(dllexport)
#else
#define ALCALL
#endif
#endif

#ifdef MODULE_IS_REMOTE

#include "Utils/include/DataUtils.h"

using namespace Utils;

string ROBOT_NAME = "Sim";

char* getCmdOption(char ** begin, char ** end, const string & option)
{
  char ** itr = find(begin, end, option);
  if (itr != end && ++itr != end)
  {
    return *itr;
  }
  return 0;
}

bool cmdOptionExists(char** begin, char** end, const string& option)
{
  return find(begin, end, option) != end;
}

int main(int argc, char* argv[]) {
  /**
   * The ip and port on which the robot is connected
   */
  int pport = 9559;
  string pip = "127.0.0.1";

  if(cmdOptionExists(argv, argv+argc, "--pip"))
  {
    pip = getCmdOption(argv, argv + argc, "--pip");
  }

  if(cmdOptionExists(argv, argv+argc, "--pport"))
  {
    DataUtils::stringToVar(getCmdOption(argv, argv + argc, "--pport"), pport);
  }

  if(cmdOptionExists(argv, argv+argc, "--robot"))
  {
    ROBOT_NAME = getCmdOption(argv, argv + argc, "--robot");
    if (!(ROBOT_NAME == "Sim" ||
        ROBOT_NAME == "Nu-11" ||
        ROBOT_NAME == "Nu-12" ||
        ROBOT_NAME == "Nu-13" ||
        ROBOT_NAME == "Nu-14" ||
        ROBOT_NAME == "Nu-15"))
    {
      ERROR("Invalid robot name: '" << ROBOT_NAME << "'. Possible names are: " << "\n1. Nu-11\n2. Nu-12\n3. Nu-13\n4. Nu-14\n5. Nu-15\n6. Sim")
      exit(1);
    }
  }

  ConfigManager::setDirPaths("Robots/" + ROBOT_NAME + "/");

  /**
   * Need this to for SOAP serialization of floats to work. Not sure 
   * what this is about as it comes from NaoQi.
   */
  setlocale(LC_NUMERIC, "C");

  /**
   * Broker name and listen port/ip
   */
  const string brokerName = "MotionModuleTests";
  int brokerPort = 54000;
  const string brokerIp = "0.0.0.0";

  /**
   * Broker definition with default configuration
   */
  boost::shared_ptr<AL::ALBroker> broker;
  try {
    broker =
    AL::ALBroker::createBroker(
      brokerName,
      brokerIp,
      brokerPort,
      pip,
      pport,
      0
    );
  } catch (...) {
    ERROR(
      "Fail to connect broker to: " + DataUtils::varToString(pip) +
      ":" + DataUtils::varToString(pport)
    )
    AL::ALBrokerManager::getInstance()->killAllBroker();
    AL::ALBrokerManager::kill();
    return 1;
  }

  /**
   * Deals with ALBrokerManager singleton (add your borker into NAOqi)
   */
  AL::ALBrokerManager::setInstance(broker->fBrokerManager.lock());
  AL::ALBrokerManager::getInstance()->addBroker(broker);

  /**
   * Loading our local module
   * Usage: createModule<our_module>(<broker_create>, <our_module>)
   */
  AL::ALModule::createModule<MotionModuleTests>(broker, "MotionModuleTests");
  while (true);
  return 0;
}
#else
extern "C"
{
  ALCALL int
  _createModule(boost::shared_ptr<AL::ALBroker> pBroker)
  {

    /**
     * Deals with ALBrokerManager singleton (add your borker into NAOqi)
     */
    AL::ALBrokerManager::setInstance(pBroker->fBrokerManager.lock());
    AL::ALBrokerManager::getInstance()->addBroker(pBroker);

    /**
     * Loading our local module
     * Usage: createModule<our_module>(<broker_create>, <our_module>)
     */
    AL::ALModule::createModule < TeamNUSTSPL > (pBroker, "TeamNUSTSPL");
    return 0;
  }

  ALCALL int
  _closeModule()
  {
    return 0;
  }
}
#endif
