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
  LOG_INFO("RobocupPenalties.initiate()")
  inBehavior = true;
}

void
RobocupPenalties::update()
{
  LOG_INFO("RobocupPenalties.update()")
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
  LOG_INFO("Setting Penalty Configuration...")
  auto gameData = IVAR(RoboCupGameControlData, PlanningModule::gameData);
  gameData.state = STATE_PLAYING;
  if ((unsigned) gameData.state == STATE_PLAYING) {
    LOG_INFO("GameCtrlState... STATE_PLAYING")
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
  LOG_INFO("RobocupPenalties.startupAction()")
  if (posture == PostureState::STAND_HANDS_BEHIND && 
      stiffness == StiffnessState::ROBOCUP) 
  {
    behaviorState = play;
  } else if (stiffness != StiffnessState::ROBOCUP) {
    if (lastSBFinished) {
      LOG_INFO("Setting stiffness high at startup.")
      auto sConfig =
        boost::make_shared <SBStiffnessConfig> (
          StiffnessState::ROBOCUP);
      setupSBRequest(sConfig);
    }
  } else if (posture != PostureState::STAND_HANDS_BEHIND) {
    if (lastMBFinished) {
      LOG_INFO("Setting posture to stand with hands behind at startup.")
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
  LOG_INFO("RobocupPenalties.goalKeeperAction()")
}

void
RobocupPenalties::strikerAction()
{
  LOG_INFO("RobocupPenalties.strikerAction()")
}

