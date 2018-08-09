/**
 * @file PlanningModule/PlanningModule.h
 *
 * This file implements the class for behavior planning.
 * All the functions and algorithms for adding intelligence to the
 * robot will be defined under this module.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include <boost/make_shared.hpp>
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "PlanningModule/include/PlanningModule.h"
#include "PlanningModule/include/PlanningConfigs.h"
#include "PlanningModule/include/PlanningRequest.h"

PlanningModule::PlanningModule(void* parent) :
  BaseModule(
    parent, 
    (unsigned) TNSPLModules::PLANNING, 
    "PlanningModule"
  )
{
}

void PlanningModule::setThreadPeriod()
{
  setPeriodMinMS(IVAR(int, planningThreadPeriod));
}

void PlanningModule::initMemoryConn()
{
  ASSERT_MSG(sharedMemory, "Shared Memory not found.");
  genericInputConnector = 
    new InputConnector(this, getModuleName() + "InputConnector");
  genericOutputConnector = 
    new OutputConnector(this, getModuleName() + "OutputConnector");
  genericInputConnector->initConnector();
  genericOutputConnector->initConnector();
}

void PlanningModule::init()
{
  pbManager = boost::make_shared<PBManager>(this);
  OVAR(PlanningState, PlanningModule::planningState) = PlanningState::STARTUP;
  OVAR(int, PlanningModule::robocupRole) = -1;
  PRINT("Starting Robot Startup Behavior.")
  PlanningRequestPtr planningRequest = 
    boost::make_shared<RequestPlanningBehavior>(
      boost::make_shared <RequestBehaviorConfig>());
  addRequest(planningRequest); // publish to itself
}

void
PlanningModule::handleRequests()
{
  if (inRequests.isEmpty())
    return;
  auto request = inRequests.queueFront();
  if (boost::static_pointer_cast <PlanningRequest>(request)) {
    auto reqId = request->getId();
    if (reqId == (unsigned)PlanningRequestIds::BEHAVIOR_REQUEST) {
      auto rpb = 
        boost::static_pointer_cast<RequestPlanningBehavior>(request);
      pbManager->manageRequest(rpb);
    } else if (reqId == (unsigned)PlanningRequestIds::KILL_BEHAVIOR) {
      pbManager->killBehavior();
    }
  }
  inRequests.popQueue();
}

void PlanningModule::mainRoutine()
{    
  pbManager->update();
  OVAR(BehaviorInfo, PlanningModule::pBehaviorInfo) = 
    pbManager->getBehaviorInfo();
}
