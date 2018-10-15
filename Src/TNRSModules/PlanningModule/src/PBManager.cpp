/**
 * @file PlanningModule/src/PBManager.cpp
 *
 * This file implements the class PBManager
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 02 Aug 2018
 */

#include "PlanningModule/include/PlanningModule.h"
#include "PlanningModule/include/PBManager.h"
#include "PlanningModule/include/PlanningBehaviors/RobotStartup/RobotStartup.h"
//#include "PlanningModule/include/PlanningBehaviors/Robocup/Robocup.h"
#include "PlanningModule/include/PlanningBehaviors/KickSequence/KickSequence.h"
#include "PlanningModule/include/PlanningBehaviors/ExternalInterface/ExternalInterface.h"

PBManager::PBManager(PlanningModule* planningModule) :
  BehaviorManager("PBManager"),
  planningModule(planningModule)
{
}

bool PBManager::makeBehavior(
  BehaviorPtr& behavior, const BehaviorConfigPtr& cfg)
{
  if (cfg->baseType != BaseBehaviorType::PLANNING)
    return false;
  if (cfg->id == (unsigned) PBIds::STARTUP) {
    behavior = 
      BehaviorPtr(RobotStartup::getType(planningModule, cfg));
  //} else if (cfg->id == (unsigned) PBIds::ROBOCUP) {
  //  behavior = 
  //    BehaviorPtr(Robocup::getType(planningModule, cfg));
  } else if (cfg->id == (unsigned) PBIds::KICK_SEQUENCE) {
    behavior = 
      BehaviorPtr(KickSequence::getType(planningModule, cfg));
  } else if (cfg->id == (unsigned) PBIds::EXTERNAL_INTERFACE) {
    behavior = 
      BehaviorPtr(ExternalInterface::getType(planningModule, cfg));
  } else {
    return false;
  }
  return true;
}
