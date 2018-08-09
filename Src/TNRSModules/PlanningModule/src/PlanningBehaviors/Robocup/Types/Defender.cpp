/**
 * @file PlanningModule/PlanningBehaviors/Defender.h
 *
 * This file declares the class Defender.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "PlanningModule/include/PlanningBehaviors/Robocup/Types/Defender.h"
#include "Utils/include/TeamPositions.h"

void
Defender::setBehaviorConfig(boost::shared_ptr<BehaviorConfig> behaviorConfig)
{
  this->behaviorConfig =
    (boost::static_pointer_cast <DefenderConfig> (behaviorConfig));
}

boost::shared_ptr<DefenderConfig>
Defender::getBehaviorCast()
{
  return boost::static_pointer_cast <DefenderConfig> (behaviorConfig);
}

void
Defender::initiate()
{
  inBehavior = true;
}

void
Defender::update()
{
  // IMPORTANT BUG
  // If robot loses its position estimate and cant find itself using landmarks
  // It gets stuck in the loop to scan the env forever. fix this.
  //
  //PRINT("In Robocup Gameplay Loop...")
  if (!inBehavior) return;
  updatePostureAndStiffness();
  setRobotIntention();
  /*if(penalisedRobot()) {
   PRINT("RobotPenalized...")
   OVAR(bool, PlanningModule::runPerceptionModules) = false;
   OVAR(bool, PlanningModule::localizeWithLastKnown) = false;
   wasPenalised = true;
   return;
   } else {
   if (wasPenalised) {
   behaviorState = getInPositions;
   OVAR(bool, PlanningModule::runPerceptionModules) = true;
   OVAR(bool, PlanningModule::robotOnSideLine) = true;
   wasPenalised = false;
   //OVAR(bool, PlanningModule::localizeWithLastKnown) = false;
   }
   }*/
  if (!reqStaticBehaviorState() || !reqMotionBehaviorState()) return;

  if (
    (IVAR(bool, PlanningModule::robotFallen) || 
    posture == PostureState::FALLING_FRONT || 
    posture == PostureState::FALLING_BACK) && !fallRecovery) 
  {
    fallenRobotAction();
  }

  // If the robot is recovering from a fall 
  if (fallRecovery) {
    if (lastMotionRequest) {
      PRINT("lastMotionRequest->id: " << lastMotionRequest->id)
      PRINT("LastMBFinished: " << lastMBFinished)
    }
    fallRecoveryAction();
  } else {
    defenderAction();
  }
}

void
Defender::finishBehaviorSafely()
{
  inBehavior = false;
}

void
Defender::defenderAction()
{
  PRINT("Executing Defender.defenderAction()...")
}
