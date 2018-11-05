/**
 * @file TeamNUSTSPL/TeamNUSTSPL.cpp
 *
 * This file implements the class TeamNUSTSPL
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 01 Feb 2017
 */

#include "CommModule/include/CommModule.h"
#include "LocalizationModule/include/LocalizationModule.h"
#include "MotionModule/include/MotionModule.h"
#include "PlanningModule/include/PlanningModule.h"
#include "SBModule/include/SBModule.h"
#include "TeamNUSTSPL/include/TeamNUSTSPL.h"
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "VisionModule/include/VisionModule.h"
#include "VisionModule/include/VisionRequest.h"

TeamNUSTSPL::TeamNUSTSPL(
  ALBrokerPtr parentBroker, const string& parentName) :
  AL::ALModule(parentBroker, parentName)
{
}

void
TeamNUSTSPL::init()
{
  LOG_INFO("Starting TeamNUSTSPL Module...")
  #ifdef MODULE_IS_LOCAL_SIMULATED
  ConfigManager::setDirPaths("Robots/Sim/");
  ConfigManager::createDirs();
  #else
  #ifndef MODULE_IS_REMOTE
  ConfigManager::createDirs();
  #endif
  #endif
  LOG_INFO("Setting up NaoQi's proxy handles...")
  memoryProxy = getParentBroker()->getMemoryProxy();
  motionProxy = getParentBroker()->getMotionProxy();
  dcmProxy = getParentBroker()->getDcmProxy();
  camProxy =
    getParentBroker()->getSpecialisedProxy<AL::ALVideoDeviceProxy>("ALVideoDevice");
  LOG_INFO("Initializing local shared memory...")
  sharedMemory = boost::make_shared<SharedMemory>();
  sharedMemory->init();
  LOG_INFO("Initializing TNRSModules...")
  setupTNRSModules();
}

void TeamNUSTSPL::setupTNRSModules()
{
  childModules.resize((unsigned) TNSPLModules::COUNT);
  childModules[(unsigned) TNSPLModules::PLANNING] =
    boost::make_shared<PlanningModule>(this, memoryProxy);
  
  childModules[(unsigned) TNSPLModules::MOTION] =
    boost::make_shared<MotionModule>(this, memoryProxy, dcmProxy, motionProxy);
  
  childModules[(unsigned) TNSPLModules::STATIC] =
    boost::make_shared<SBModule>(this, memoryProxy, dcmProxy, motionProxy);
  
  //childModules[(unsigned) TNSPLModules::LOCALIZATION] =
  //  boost::make_shared<LocalizationModule>(this);
    
  //childModules[(unsigned) TNSPLModules::VISION] =
  //  boost::make_shared<VisionModule>(this, camProxy);
  
  #ifdef MODULE_IS_REMOTE
  if (SAVE_IMAGES != -1) {
    auto vRequest = boost::make_shared<SwitchVision>(true);
    BaseModule::publishModuleRequest(vRequest);
    auto sliRequest = boost::make_shared<SwitchLogImages>(true, SAVE_IMAGES);
    BaseModule::publishModuleRequest(sliRequest);
  }
  if (PROJECT_FIELD == 1) {
    auto vRequest = boost::make_shared<SwitchVision>(true);
    BaseModule::publishModuleRequest(vRequest);
    auto sfpRequest = boost::make_shared<SwitchFieldProjection>(true);
    BaseModule::publishModuleRequest(sfpRequest);
  }
  if (USE_LOGGED_IMAGES == 1) {
    auto vRequest = boost::make_shared<SwitchVision>(true);
    BaseModule::publishModuleRequest(vRequest);
    auto uliRequest = boost::make_shared<SwitchUseLoggedImages>(true);
    BaseModule::publishModuleRequest(uliRequest);
  }
  #endif
  
  //childModules[(unsigned)TNSPLModules::COMM] =
  //  boost::make_shared<CommModule>(this);
  
  for (size_t i = 0; i < childModules.size(); ++i) {
    if (childModules[i]) {
      childModules[i]->setLocalSharedMemory(sharedMemory);
      childModules[i]->setupModule();
    }
  }
  for (size_t i = 0; i < childModules.size(); ++i) {
    if (childModules[i])
      childModules[i]->startModule();
  }
}
