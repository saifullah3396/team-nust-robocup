/**
 * @file SBModule/src/SBManager.cpp
 *
 * This file implements the class SBManager
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 02 Aug 2018
 */

#include "SBModule/include/SBModule.h"
#include "SBModule/include/SBManager.h"
#include "SBModule/include/StiffnessModule/StiffnessModule.h"
#include "SBModule/include/LedsModule/LedsModule.h"
#include "SBModule/include/WhistleDetector/WhistleDetector.h"

SBManager::SBManager(SBModule* sbModule) :
  BehaviorManager("SBManager"),
  sbModule(sbModule)
{
}

bool
SBManager::makeBehavior(
  BehaviorPtr& behavior, const BehaviorConfigPtr& cfg)
{
  if (cfg->baseType != BaseBehaviorType::STATIC)
    return false;
  if (cfg->id == (unsigned) SBIds::STIFFNESS) {
    behavior = BehaviorPtr(StiffnessModule::getType(sbModule, cfg));
  } else if (cfg->id == (unsigned) SBIds::LEDS) {
    behavior = BehaviorPtr(LedsModule::getType(sbModule, cfg));
  } else if (cfg->id == (unsigned) SBIds::WHISTLE_DETECTOR) {
    behavior = BehaviorPtr(WhistleDetector::getType(sbModule, cfg));
  } else {
    return false;
  }
  return true;
}
