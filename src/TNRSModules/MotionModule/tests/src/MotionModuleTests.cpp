/**
 * @file MotionModuleTests/MotionModuleTests.cpp
 *
 * This file implements the class MotionModuleTests
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 01 Feb 2017
 */

#include "ControlModule/include/ControlModule.h"
#include "MotionModule/include/MotionModule.h"
#include "MotionModule/tests/include/MotionModuleTests.h"

MotionModuleTests::MotionModuleTests(ALBrokerPtr parentBroker,
  const string& parentName) :
  AL::ALModule(parentBroker, parentName)
{
}

void
MotionModuleTests::init()
{
  LOG_INFO("Starting MotionModuleTests...")
  LOG_INFO("Setting up NaoQi's Proxies...")
  memoryProxy = getParentBroker()->getMemoryProxy();
  motionProxy = getParentBroker()->getMotionProxy();
  dcmProxy = getParentBroker()->getDcmProxy();
  LOG_INFO("Initializing memory...")
  sharedMemory = boost::make_shared<SharedMemory>();
  sharedMemory->init();
  LOG_INFO("SharedMemory Initialized.")
  motionProxy->wakeUp();
  childModules.push_back(
    boost::make_shared < ControlModule > (this, memoryProxy, dcmProxy));
  childModules.push_back(
    boost::make_shared < MotionModule > (this, motionProxy));
  for (int i = 0; i < childModules.size(); ++i) {
    childModules[i]->setLocalSharedMemory(sharedMemory);
    childModules[i]->setupModule();
  }
  
  for (int i = 0; i < childModules.size(); ++i)
    childModules[i]->startModule();
}
