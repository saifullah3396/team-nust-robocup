/**
 * @file PlanningModule/PlanningBehaviors/Attacker.h
 *
 * This file declares the class Attacker.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "PlanningModule/include/PlanningBehaviors/Robocup/Types/Attacker.h"
#include "Utils/include/TeamPositions.h"

void
Attacker::setBehaviorConfig(
  boost::shared_ptr<BehaviorConfig> behaviorConfig)
{
  this->behaviorConfig =
    (boost::static_pointer_cast <AttackerConfig> (behaviorConfig));
}

boost::shared_ptr<AttackerConfig>
Attacker::getBehaviorCast()
{
  return boost::static_pointer_cast <AttackerConfig> (behaviorConfig);
}

void
Attacker::initiate()
{
  PRINT("Attacker.initiated()")
  inBehavior = true;
}

void
Attacker::update()
{
  PRINT("Attacker.update()")
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
    attackerAction();
  }
}

void
Attacker::finishBehaviorSafely()
{
  inBehavior = false;
}

void
Attacker::attackerAction()
{
  PRINT("Executing Attacker.attackerAction()...")
}
