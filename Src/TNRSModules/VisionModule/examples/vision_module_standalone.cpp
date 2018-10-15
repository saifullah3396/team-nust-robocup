#include <iostream>
#include <exception>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <alproxies/almotionproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include "MotionModule/include/MotionModule.h"
#include "VisionModule/include/VisionModule.h"
#include "VisionModule/include/VisionRequest.h"
#include "Utils/include/ConfigManager.h"
#include "TNRSBase/include/BaseIncludes.h"
#include "Utils/include/ArgParser.h"
#include "Utils/include/Exceptions/ArgParseException.h"

using namespace std;

typedef boost::shared_ptr<AL::ALMotionProxy> ALMotionProxyPtr;
typedef boost::shared_ptr<AL::ALMemoryProxy> ALMemoryProxyPtr;
typedef boost::shared_ptr<AL::ALVideoDeviceProxy> ALVideoDeviceProxyPtr;
typedef boost::shared_ptr<AL::DCMProxy> ALDCMProxyPtr;

int USE_LOGGED_IMAGES = 0;
int SAVE_IMAGES = -1;
int PROJECT_FIELD = 0;
string ROBOT_NAME = "Sim";

int
main(int argc, char* argv[])
{
  /**
   * The ip and port on which the robot is connected
   */
  int pport = 9559;
  string pip = "127.0.0.1";

  if(ArgParser::cmdOptionExists(argv, argv+argc, "--pip"))
  {
    pip = ArgParser::getCmdOption(argv, argv + argc, "--pip");
    try {
	  if (!ArgParser::cmdOptionExists(argv, argv+argc, "--robot"))
	    throw "Please give the argument --robot";
	} catch(const exception& e) {
	  ERROR(e.what())
	}
  }

  if(ArgParser::cmdOptionExists(argv, argv+argc, "--pport"))
  {
    DataUtils::stringToVar(ArgParser::getCmdOption(argv, argv + argc, "--pport"), pport);
  }

  if(ArgParser::cmdOptionExists(argv, argv+argc, "--use-logged-images"))
  {
    USE_LOGGED_IMAGES = 1;
  }

  if(ArgParser::cmdOptionExists(argv, argv+argc, "--test-projection"))
  {
    PROJECT_FIELD = 1;
  }

  if(ArgParser::cmdOptionExists(argv, argv+argc, "--save-images"))
  {
    string option = ArgParser::getCmdOption(argv, argv + argc, "--save-images");
    if (option == "Top") {
      SAVE_IMAGES = 0;
    } else if (option == "Bottom") {
      SAVE_IMAGES = 1;
    }
    if(SAVE_IMAGES != 0 && SAVE_IMAGES != 1) {
      ERROR("Please add argument value Top/Bottom for required camera input.")
      exit(1);
    }
    PRINT("Press ENTER key to start receiving image input from the concerned camera.\n" <<
      "Please save images of a checker board from the concerned camera for calibration.\n" <<
      "Before Continuing, don't forget to update the file: \nConfig/CameraCalibration.xml.\n");
    if(cin.get() == '\n');
  }

  if(ArgParser::cmdOptionExists(argv, argv+argc, "--robot"))
  {
    ROBOT_NAME = ArgParser::getCmdOption(argv, argv + argc, "--robot");
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

	//! Path to robot's configuration files
  ConfigManager::setDirPaths("Robots/" + ROBOT_NAME + "/");
  
  //! Setting up shared memory
  SharedMemoryPtr sharedMemory;
  sharedMemory = boost::make_shared<SharedMemory>();
  sharedMemory->init();

  //! Setting up Naoqi camera proxy
  ALMotionProxyPtr motionProxy =
	boost::make_shared <AL::ALMotionProxy> (pip, pport);
  //! Setting up Naoqi camera proxy
  ALVideoDeviceProxyPtr camProxy =
    boost::make_shared <AL::ALVideoDeviceProxy> (pip, pport);
  //! Pointer to dummy parent thread which is this one.
  int* dummyParent;
  //! Vision module depends on robot kinematics
  //! Setting up motion module
  auto mModule =
    boost::make_shared < MotionModule > (dummyParent, motionProxy);
  mModule->setLocalSharedMemory(
    sharedMemory);
  mModule->setupModule();
  //! Starting up motion module thread
  mModule->startModule();
  
  auto vModule =
    boost::make_shared<VisionModule>(dummyParent, camProxy);
  //! Setting up vision module
  vModule->setLocalSharedMemory(sharedMemory);
  vModule->setupModule();
  
  auto vRequest = boost::make_shared<SwitchVision>(true);
    BaseModule::publishModuleRequest(vRequest);
  
  if (SAVE_IMAGES != -1) {
    auto sliRequest = boost::make_shared<SwitchLogImages>(true, SAVE_IMAGES);
    BaseModule::publishModuleRequest(sliRequest);
  }
  if (PROJECT_FIELD == 1) {
    auto sfpRequest = boost::make_shared<SwitchFieldProjection>(true);
    BaseModule::publishModuleRequest(sfpRequest);
  }
  if (USE_LOGGED_IMAGES == 1) {
    auto uliRequest = boost::make_shared<SwitchUseLoggedImages>(true);
    BaseModule::publishModuleRequest(uliRequest);
  }
  
  //! Starting up vision module thread
  vModule->startModule();
  
  while(true);
  return 1;
}
