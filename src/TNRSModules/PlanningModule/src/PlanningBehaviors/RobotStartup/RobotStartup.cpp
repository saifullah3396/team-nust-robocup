/**
 * @file PlanningModule/src/PlanningBehaviors/RobotStartup/RobotStartup.cpp
 *
 * This file implements the class RobotStartup
 *
 * @author <A href="mailto:saifullah3396@rsail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "PlanningModule/include/PlanningBehaviors/RobotStartup/RobotStartup.h"
#include "PlanningModule/include/PlanningBehaviors/RobotStartup/Types/RequestBehavior.h"

boost::shared_ptr<RobotStartup> RobotStartup::getType(
  PlanningModule* planningModule, const BehaviorConfigPtr& cfg) 
{ 
  RobotStartup* rs;
  switch (cfg->type) {
      case (unsigned) PBStartupTypes::REQUEST_BEHAVIOR: 
        rs = new RequestBehavior(planningModule, cfg); break;
      default: rs = new RequestBehavior(planningModule, cfg); break;
  }
  return boost::shared_ptr<RobotStartup>(rs);
}

PBStartupConfigPtr RobotStartup::getBehaviorCast()
{
  return boost::static_pointer_cast <PBStartupConfig> (config);
}
