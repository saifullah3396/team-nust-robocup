/**
 * @file PlanningModule/PlanningBehaviors/ExternalInterface.h
 *
 * This file declares the class ExternalInterface
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "PlanningModule/include/PlanningBehaviors/ExternalInterface/ExternalInterface.h"
#include "PlanningModule/include/PlanningBehaviors/ExternalInterface/Types/NIHACognition.h"
#include "PlanningModule/include/PlanningBehaviors/ExternalInterface/Types/UserRequestsHandler.h"

boost::shared_ptr<ExternalInterface> ExternalInterface::getType(
  PlanningModule* planningModule, const BehaviorConfigPtr& cfg) 
{ 
  ExternalInterface* cm;
  switch (cfg->type) {
      case (unsigned) PBExternalInterfaceTypes::NIHA_COGNITION: 
        cm = new NIHACognition(planningModule, cfg); break;
      case (unsigned) PBExternalInterfaceTypes::USER_REQ_HANDLER: 
        cm = new UserRequestsHandler(planningModule, cfg); break;
      default: cm = new UserRequestsHandler(planningModule, cfg); break;
  }
  return boost::shared_ptr<ExternalInterface>(cm);
}

PBExternalInterfaceConfigPtr ExternalInterface::getBehaviorCast()
{
  return boost::static_pointer_cast <PBExternalInterfaceConfig> (config);
}
