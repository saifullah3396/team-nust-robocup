/**
 * @file PlanningModule/PlanningBehaviors/KickSequence.h
 *
 * This file declares the class KickSequence.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "PlanningModule/include/PlanningBehaviors/KickSequence/KickSequence.h"
#include "PlanningModule/include/PlanningBehaviors/KickSequence/Types/BallIntercept.h"

boost::shared_ptr<KickSequence> KickSequence::getType(
  PlanningModule* planningModule, const BehaviorConfigPtr& cfg) 
{ 
  KickSequence* ks;
  switch (cfg->type) {
      case (unsigned) PBKickSequenceTypes::BALL_INTERCEPT: 
        ks = new BallIntercept(planningModule, cfg); break;
      default: ks = new BallIntercept(planningModule, cfg); break;
  }
  return boost::shared_ptr<KickSequence>(ks);
}

PBKickSequenceConfigPtr KickSequence::getBehaviorCast()
{
  return boost::static_pointer_cast <PBKickSequenceConfig> (config);
}
