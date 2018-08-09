/**
 * @file PlanningModule/PlanningBehaviors/RobocupPenalties.h
 *
 * This file declares the class RobocupPenalties.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "MotionModule/include/MotionConfigs/MBPostureConfig.h"
#include "PlanningModule/include/PlanningBehaviors/Robocup/Types/RobocupPenalties.h"
#include "SBModule/include/SBConfigs.h"

void
RobocupPenalties::setBehaviorConfig(
  boost::shared_ptr<BehaviorConfig> behaviorConfig)
{
  this->behaviorConfig =
    (boost::static_pointer_cast < PenaltiesConfig > (behaviorConfig));
}

boost::shared_ptr<PenaltiesConfig>
RobocupPenalties::getBehaviorCast()
{
  return boost::static_pointer_cast < PenaltiesConfig > (behaviorConfig);
}

void
RobocupPenalties::initiate()
{
  PRINT("RobocupPenalties.initiate()")
  inBehavior = true;
}

void
RobocupPenalties::update()
{
  PRINT("RobocupPenalties.update()")
  if (!reqStaticBehaviorState() || !reqMotionBehaviorState()) return;
  updatePostureAndStiffness();

  if (behaviorState == penaltyCfg) {
    penaltyCfgAction();
  } else if (behaviorState == startup) {
    startupAction();
  } else if (behaviorState == play) {
    if (striker) strikerAction();
    else goalKeeperAction();
  }
}

void
RobocupPenalties::finishBehaviorSafely()
{
  inBehavior = false;
}

void
RobocupPenalties::penaltyCfgAction()
{
  PRINT("Setting Penalty Configuration...")
  auto gameData = IVAR(RoboCupGameControlData, PlanningModule::gameData);
  gameData.state = STATE_PLAYING;
  if ((unsigned) gameData.state == STATE_PLAYING) {
    PRINT("GameCtrlState... STATE_PLAYING")
    auto secState = (unsigned) gameData.secondaryState;
    auto kickOffTeam = (unsigned) gameData.kickOffTeam;
    auto& ourTeamNumber = IVAR(int, PlanningModule::teamNumber);
    if (secState == STATE2_PENALTYSHOOT) {
      if (kickOffTeam == ourTeamNumber) {
        striker = true; // striker
      } else {
        striker = false; // goalkeeper
      }
      behaviorState = startup;
    }
  }
}

void 
RobocupPenalties::startupAction()
{
  PRINT("RobocupPenalties.startupAction()")
  if (posture == PostureState::STAND_HANDS_BEHIND && 
      stiffness == StiffnessState::ROBOCUP) 
  {
    behaviorState = play;
  } else if (stiffness != StiffnessState::ROBOCUP) {
    if (lastSBFinished) {
      PRINT("Setting stiffness high at startup.")
      auto sConfig =
        boost::make_shared <SBStiffnessConfig> (
          StiffnessState::ROBOCUP);
      setupSBRequest(sConfig);
    }
  } else if (posture != PostureState::STAND_HANDS_BEHIND) {
    if (lastMBFinished) {
      PRINT("Setting posture to stand with hands behind at startup.")
      auto pConfig = 
        boost::make_shared<MBPostureConfig>(
          PostureState::STAND_HANDS_BEHIND, 2.f);
      setupMBRequest(pConfig);
    }
  }
}

void
RobocupPenalties::goalKeeperAction()
{
  PRINT("RobocupPenalties.goalKeeperAction()")
}

void
RobocupPenalties::strikerAction()
{
  PRINT("RobocupPenalties.strikerAction()")
}

