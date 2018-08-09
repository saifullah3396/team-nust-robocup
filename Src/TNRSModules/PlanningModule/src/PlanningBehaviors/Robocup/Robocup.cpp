/**
 * @file PlanningModule/PlanningBehaviors/Robocup.h
 *
 * This file implements the class Robocup.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "MotionModule/include/MotionConfigs/MBPostureConfig.h"
#include "MotionModule/include/MotionConfigs/MBGetupConfig.h"
#include "PlanningModule/include/PlanningBehaviors/Robocup/Robocup.h"
#include "PlanningModule/include/PlanningBehaviors/Robocup/Types/RobocupSetup.h"
#include "PlanningModule/include/PlanningBehaviors/Robocup/Types/GoalKeeper.h"
#include "SBModule/include/SBConfigs.h"
#include "Utils/include/TeamPositions.h"

boost::shared_ptr<Robocup> Robocup::getType(
  PlanningModule* planningModule, const unsigned& type) 
{ 
  Robocup* r;
  switch (type) {
      case (unsigned) PBRobocupTypes::ROBOCUP_SETUP: 
        r = new RobocupSetup(planningModule); break;
      case (unsigned) PBRobocupTypes::GOAL_KEEPER: 
        r = new GoalKeeper(planningModule); break;
      default: r = new RobocupSetup(planningModule); break;
  }
  return boost::shared_ptr<Robocup>(r);
}

void
Robocup::setBehaviorConfig(
  boost::shared_ptr<BehaviorConfig> behaviorConfig)
{
  this->behaviorConfig =
    (boost::static_pointer_cast <PBRobocupConfig> (behaviorConfig));
}

boost::shared_ptr<PBRobocupConfig>
Robocup::getBehaviorCast()
{
  return boost::static_pointer_cast <PBRobocupConfig> (behaviorConfig);
}

void
Robocup::setRobotSuggestions()
{
  /*auto& teamRobots = IVAR(vector<TeamRobot>, PlanningModule::teamRobots);
   for (int i = 0; i < teamRobots.size(); ++i) {
   // Wait for team to get localized
   if (teamRobots[i].intention == ) { 
   ++teamRobotsLost;
   continue;
   }
   float dist = 
   MathsUtils::dist(
   robotState.x, robotState.y,
   teamRobots[i].pose.x, teamRobots[i].pose.y
   );
   // Check if another team member has overlapping position 
   if (dist < 0.2) {
   reqRelocalization();
   teamRobotOverlap = true;
   }
   }*/
}

void
Robocup::fallenRobotAction()
{
  PRINT("Executing Robocup.fallenRobotAction()...")
  if (lastMBStarted) lastMotionRequest->kill();
  // Turn off perception modules and reset when robot is stable again
  OVAR(bool, PlanningModule::runVisionModule) = false;
  OVAR(bool, PlanningModule::runLocalizationModule) = false;
  fallRecovery = true;
}

void
Robocup::fallRecoveryAction()
{
  PRINT("Executing Robocup.fallRecoveryAction()...")
  bool& fallen = IVAR(bool, PlanningModule::robotFallen);
  if (fallen) {
    if (posture == PostureState::FALL_FRONT) {
      getupFront();
    } else if (posture == PostureState::FALL_BACK) {
      getupBack();
    } else if (posture == PostureState::FALL_SIT) {
      getupSit();
    }
  } else {
    float postureTime = 2.f;
    if (posture == PostureState::FALLING_FRONT) {
      postureTime = 0.5f;
    } else if (posture == PostureState::FALLING_BACK) {
      postureTime = 0.5f;
    }
    if (posture != PostureState::STAND) {
      if (lastMBFinished) {
        PRINT("Setting posture to stand at fallen.")
        auto pConfig = 
          boost::make_shared<MBPostureConfig>(
            PostureState::STAND, postureTime);
        setupMBRequest(pConfig);
      }
    } else {
      OVAR(bool, PlanningModule::localizeWithLastKnown) = true;
      OVAR(bool, PlanningModule::runVisionModule) = true;
      OVAR(bool, PlanningModule::runLocalizationModule) = true;
      fallRecovery = false;
    }
  }
}

void
Robocup::setRobotIntention()
{
  auto& role = OVAR(int, PlanningModule::robocupRole);
  auto& localized = IVAR(bool, PlanningModule::robotLocalized);
  if (localized || role == -1) {
    if (role == (int)RobocupRole::GOALKEEPER)
    OVAR(int, PlanningModule::robotIntention) = 1;
    else if (role == (int)RobocupRole::DEFENDER || role == (int)RobocupRole::DEFENSE_SUPPORT)
    OVAR(int, PlanningModule::robotIntention) = 2;
    else if (role == (int)RobocupRole::OFFENSE_SUPPORT || role == (int)RobocupRole::ATTACKER)
    OVAR(int, PlanningModule::robotIntention) = 3;
  } else {
    OVAR(int, PlanningModule::robotIntention) = 4; // robot lost
  }
}

void
Robocup::getupBack()
{
  PRINT("Executing Robocup.getupBack()...")
  //PRINT("Posture: " << (int)posture)
  //PRINT("Stiffness: " << (int)stiffness)
  if (!readyToGetup) {
    //PRINT("Getting ready to get up...")
    if (stiffness != StiffnessState::GETUP) {
      //PRINT("lastSBFinished: " << (int)lastSBFinished)
      if (lastSBFinished) {
        //PRINT("Setting StiffnessState to GETUP...")
        auto sConfig =
          boost::make_shared < SBStiffnessConfig > (StiffnessState::GETUP);
        setupSBRequest(sConfig);
      }
    } else {
      readyToGetup = true;
    }
  } else {
    if (lastMBFinished) {
      //PRINT("Starting Getup Sequence from Back...")
      auto getupConfig =
        boost::make_shared<KFMGetupConfig>(KeyFrameGetupTypes::BACK);
      setupMBRequest(getupConfig);
      readyToGetup = false;
    }
  }
}

void
Robocup::getupFront()
{
  PRINT("Executing Robocup.getupFront()...")
  //PRINT("Posture: " << (int)posture)
  //PRINT("Stiffness: " << (int)stiffness)
  if (!readyToGetup) {
    //PRINT("Getting ready to get up...")
    if (stiffness != StiffnessState::GETUP) {
      if (lastSBFinished) {
        //PRINT("Setting StiffnessState to GETUP...")
        auto sConfig =
          boost::make_shared < SBStiffnessConfig > (StiffnessState::GETUP);
        setupSBRequest(sConfig);
      }
    } else {
      readyToGetup = true;
    }
  } else {
    if (lastMBFinished) {
      //PRINT("Starting Getup Sequence from Front...")
      auto getupConfig =
        boost::make_shared<KFMGetupConfig>(KeyFrameGetupTypes::FRONT);
      setupMBRequest(getupConfig);
      readyToGetup = false;
    }
  }
}

void
Robocup::getupSit()
{
}
