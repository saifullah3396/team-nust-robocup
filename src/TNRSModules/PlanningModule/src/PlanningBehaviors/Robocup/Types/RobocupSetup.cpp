/**
 * @file PlanningModule/PlanningBehaviors/RobocupSetupSetup.h
 *
 * This file implements the class RobocupSetupSetup.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 27 Jul 2018
 */

#include "PlanningModule/include/PlanningBehaviors/Robocup/Types/RobocupSetup.h"
#include "MotionModule/include/MotionConfigs/MBPostureConfig.h"
#include "MotionModule/include/MotionConfigs/MBHeadControlConfig.h"
#include "SBModule/include/SBConfigs.h"
#include "Utils/include/RobocupRole.h"
#include "Utils/include/TeamPositions.h"

void
RobocupSetup::setBehaviorConfig(
  boost::shared_ptr<BehaviorConfig> behaviorConfig)
{
  this->behaviorConfig =
    (boost::static_pointer_cast <RobocupSetupConfig> (behaviorConfig));
}

boost::shared_ptr<RobocupSetupConfig>
RobocupSetup::getBehaviorCast()
{
  return boost::static_pointer_cast <RobocupSetupConfig> (behaviorConfig);
}

void
RobocupSetup::initiate()
{
  LOG_INFO("RobocupSetup.initiate()")
  inBehavior = true;
}

void
RobocupSetup::update()
{
  LOG_INFO("RobocupSetup.update()")
  // IMPORTANT BUG
  // If robot loses its position estimate and cant find itself using landmarks
  // It gets stuck in the loop to scan the env forever. fix this.
  //
  //LOG_INFO("In RobocupSetup Gameplay Loop...")
  updatePostureAndStiffness();
  setRobotIntention();
  /*if(penalisedRobot()) {
   LOG_INFO("RobotPenalized...")
   OVAR(bool, PlanningModule::runVisionModule) = false;
   OVAR(bool, PlanningModule::runLocalizationModule) = false;
   OVAR(bool, PlanningModule::localizeWithLastKnown) = false;
   wasPenalised = true;
   return;
   } else {
   if (wasPenalised) {
   behaviorState = getInPositions;
   OVAR(bool, PlanningModule::runVisionModule) = true;
   OVAR(bool, PlanningModule::runLocalizationModule) = true;
   OVAR(bool, PlanningModule::robotOnSideLine) = true;
   wasPenalised = false;
   //OVAR(bool, PlanningModule::localizeWithLastKnown) = false;
   }
   }*/
  if (!reqStaticBehaviorState() || !reqMotionBehaviorState()) return;

  if ((IVAR(bool, PlanningModule::robotFallen) || posture == PostureState::FALLING_FRONT || posture == PostureState::FALLING_BACK) && !fallRecovery) {
    fallenRobotAction();
  }

  // Just for debugging individual behaviors bypassing this behavior
  //behaviorState = gameplaySequence;
  // If the robot is recovering from a fall 
  if (fallRecovery) {
    if (lastMotionRequest) {
      LOG_INFO("lastMotionRequest->id: " << lastMotionRequest->id)
      LOG_INFO("LastMBFinished: " << lastMBFinished)
    }
    fallRecoveryAction();
  } else {
    if (behaviorState == startup) {
      startupAction();
    } else if (behaviorState == robocupCfg) {
      cfgHandlingAction();
    } else if (behaviorState == readySequence) {
      readySequenceAction();
    } else if (behaviorState == getInPositions) {
      getInPositionsAction();
    } else if (behaviorState == setSequence) {
      setSequenceAction();
    } else if (behaviorState == gameplaySequence) {
      gameplaySequenceAction();
    }
  }
}

void
RobocupSetup::finishBehaviorSafely()
{
  inBehavior = false;
}

void
RobocupSetup::startupAction()
{
  LOG_INFO("Executing RobocupSetup.startupAction()...")
  if (posture == PostureState::STAND && 
      stiffness == StiffnessState::ROBOCUP) 
  {
    LOG_INFO("Setting RobocupSetup Gameplay Startup...")
    behaviorState = robocupCfg;
  } else if (stiffness != StiffnessState::ROBOCUP) {
    if (lastSBFinished) {
      LOG_INFO("Setting stiffness high at startup.")
      auto sConfig =
        boost::make_shared <SBStiffnessConfig> (
          StiffnessState::ROBOCUP);
      setupSBRequest(sConfig);
    }
  } else if (posture != PostureState::STAND) {
    if (lastMBFinished) {
      LOG_INFO("Setting posture to stand at startup.")
      auto pConfig = 
        boost::make_shared<MBPostureConfig>(
          PostureState::STAND, 2.f);
      setupMBRequest(pConfig);
    }
  }
}

void
RobocupSetup::cfgHandlingAction()
{
  LOG_INFO("Executing RobocupSetup.cfgHandlingAction()...")
  //LOG_INFO("Setting RobocupSetup Configuration...")
  auto& gameData = IVAR(RoboCupGameControlData, PlanningModule::gameData);
  ///Example states for checking penalty shootout behavior
  ///gameData.state = STATE_SET;
  ///gameData.secondaryState = STATE2_PENALTYSHOOT;
  ///gameData.kickOffTeam = 30;
  ///Example states for checking gameplay behavior
  gameData.state = STATE_READY;
  gameData.secondaryState = STATE2_NORMAL;
  gameData.kickOffTeam = 30;
  if ((unsigned) gameData.state == STATE_INITIAL) {
    LOG_INFO("GameCtrlState... STATE_INITIAL")
  } else if ((unsigned) gameData.state == STATE_READY) {
    LOG_INFO("GameCtrlState... STATE_READY")
    behaviorState = readySequence;
    OVAR(bool, PlanningModule::runVisionModule) = true;
    OVAR(bool, PlanningModule::runLocalizationModule) = true;
    OVAR(bool, PlanningModule::robotOnSideLine) = true;
  } else if ((unsigned) gameData.state == STATE_SET) {
    LOG_INFO("GameCtrlState... STATE_SET")
    auto secState = (unsigned) gameData.secondaryState;
    auto kickOffTeam = (unsigned) gameData.kickOffTeam;
    auto& ourTeamNumber = IVAR(int, PlanningModule::teamNumber);
    if (secState == STATE2_PENALTYSHOOT) {
      if (kickOffTeam == ourTeamNumber) {
        LOG_INFO("[GameCtrlSecState... Penalty Striker")
        killallBehaviors();
         // auto planConfig =
         //   boost::make_shared < PBPenaltiesConfig > (PBPenaltiesTypes::PENALTY_STRIKER);
         // setupChildBehaviorRequest(planConfig);
      } else {
        LOG_INFO("GameCtrlSecState... Penalty GoalKeeper")
        killallBehaviors();
         //auto planConfig =
         //   boost::make_shared < PBPenaltiesConfig > (PBPenaltiesTypes::PENALTY_GOALKEEPER);
         // setupChildBehaviorRequest(planConfig);
      }
    }
  }
}

void
RobocupSetup::readySequenceAction()
{
  LOG_INFO("Executing RobocupSetup.readySequenceAction()...")
  auto& localized = IVAR(bool, PlanningModule::robotLocalized);
  if (!localized) {
    LOG_INFO("goalInfo.found: " << IVAR(GoalInfo, PlanningModule::goalInfo).found)
    LOG_INFO("goalInfo.ours: " << IVAR(GoalInfo, PlanningModule::goalInfo).ours)
    LOG_INFO(
      "goalInfo.leftPost: " << IVAR(GoalInfo, PlanningModule::goalInfo).leftPost)
    LOG_INFO(
      "goalInfo.rightPost: " << IVAR(GoalInfo, PlanningModule::goalInfo).rightPost)

    if (lastMBFinished) {
      LOG_INFO("Setting Find Goal Behavior...")
      auto mConfig =
        boost::make_shared <HeadTargetSearchConfig> (
          HeadTargetTypes::GOAL);
      // Robot is on sidelines with other robots so keep scan range minimum.
      mConfig->scanMaxYaw = 75.0 * M_PI / 180;
      setupMBRequest(mConfig);
    }
  } else {
    bool teamRobotOverlap = false;
    auto& robotState = IVAR(RobotPose2D<float>, PlanningModule::robotPose2D);
    auto& teamRobots = IVAR(vector<TeamRobot>, PlanningModule::teamRobots);
    unsigned teamRobotsLost = 0;
    for (int i = 0; i < teamRobots.size(); ++i) {
      // Wait for team to get localized
      if (teamRobots[i].positionConfidence < 50) {
        ++teamRobotsLost;
        continue;
      }
      float dist = MathsUtils::dist(
        robotState.x,
        robotState.y,
        teamRobots[i].pose.x,
        teamRobots[i].pose.y);
      // Check if another team member has overlapping position 
      if (dist < 0.2) {
        reqRelocalization();
        teamRobotOverlap = true;
      }
    }

    cout << "teamRobotsLost: " << teamRobotsLost << endl;
    teamRobotsLost = 2;
    if (!teamRobotOverlap && teamRobotsLost <= 2) {
      auto& gameData = IVAR(RoboCupGameControlData, PlanningModule::gameData);
      auto kickOffTeam = (unsigned) gameData.kickOffTeam;
      auto& ourTeamNumber = IVAR(int, PlanningModule::teamNumber);
      int robocupRole = (int) RobocupRole::GOALKEEPER;
      double smallestDist = 1e3;
      if (kickOffTeam == ourTeamNumber) {
        for (size_t i = (int) RobocupRole::GOALKEEPER;
          i < (int) RobocupRole::COUNT; ++i) {
          double dist = MathsUtils::dist(
            sidePositionsAtt[i].x,
            sidePositionsAtt[i].y,
            robotState.x,
            robotState.y);
          if (dist < smallestDist) {
            smallestDist = dist;
            robocupRole = i;
          }
        }
      } else {
        for (size_t i = (int) RobocupRole::GOALKEEPER;
          i < (int) RobocupRole::COUNT; ++i) {
          double dist = MathsUtils::dist(
            sidePositionsDef[i].x,
            sidePositionsDef[i].y,
            robotState.x,
            robotState.y);
          if (dist < smallestDist) {
            smallestDist = dist;
            robocupRole = i;
          }
        }
      }
      // Chnage this varaible to unsigned or RobocupRole
      OVAR(int, PlanningModule::robocupRole) = robocupRole;
      setRobotIntention();
      OVAR(bool, PlanningModule::localizeWithLastKnown) = true;
      behaviorState = getInPositions;
    }
  }
}

void
RobocupSetup::getInPositionsAction()
{
  LOG_INFO("Executing RobocupSetup.getInPositionsAction()...")
  auto& gameData = IVAR(RoboCupGameControlData, PlanningModule::gameData);
  // If the ready sequence time is up and game controller sent set 
  // command
  if ((unsigned) gameData.state == STATE_SET) {
    behaviorState = setSequence;
  } else {
    // Else keep trying to get in correct position
    auto& localized = IVAR(bool, PlanningModule::robotLocalized);
    if (!localized) {
      LOG_INFO("Robot not localized. Trying to scan and get localized...")
      OVAR(bool, PlanningModule::localizeWithLastKnown) = true;
      if (lastMBFinished) {
        auto mConfig =
          boost::make_shared <HeadTargetSearchConfig> (
            HeadTargetTypes::LANDMARKS);
        // If robot is on Sidelines alone. Better to find T or L corners too. 
        if (OVAR(bool, PlanningModule::robotOnSideLine)) mConfig->scanMaxYaw =
          100.0 * M_PI / 180;
        else // If robot is in the field.
        mConfig->scanMaxYaw = 119.0 * M_PI / 180;
        setupMBRequest(mConfig);
      }
    } else {
      if (lastMBStarted && lastMotionRequest->id == (unsigned) MBIds::HEAD_CONTROL) {
        lastMotionRequest->kill();
      }
      LOG_INFO("Robot localized")
      if (lastMotionRequest) {
        LOG_INFO("lastMotionRequest->id: " << lastMotionRequest->id)
        LOG_INFO("LastMBFinished: " << lastMBFinished)
      }
      // After the walk finishes reset the localizer to find its 
      // position estimate from landmarks
      if (lastMBFinished && lastMotionRequest->id == (unsigned) MBIds::MOVEMENT) {
        LOG_INFO("Walk finished. Resetting localizer...")
        reqRelocalization();
        OVAR(bool, PlanningModule::localizeWithLastKnown) = true;
      } else if (lastMBFinished) {
        LOG_INFO("Setting walk with target...")
        RobotPose2D<float> target;
        if (setSequenceFinished) {
          auto& role = OVAR(int, PlanningModule::robocupRole);
          if (role == (int)RobocupRole::GOALKEEPER) {
            target = keeperPositionInGame[0];
          } else if (role == (int)RobocupRole::DEFENDER || role == (int)RobocupRole::DEFENSE_SUPPORT) {
            auto& robotState =
              IVAR(RobotPose2D<float>, PlanningModule::robotPose2D);
            double smallestDist = 1e3;
            for (int i = 0; i < 2; ++i) {
              double dist = MathsUtils::dist(
                defPositionsInGame[i].x,
                defPositionsInGame[i].y,
                robotState.x,
                robotState.y);
              if (dist < smallestDist) {
                smallestDist = dist;
                target = defPositionsInGame[i];
              }
            }
          } else if (role == (int)RobocupRole::OFFENSE_SUPPORT || role == (int)RobocupRole::ATTACKER) {
            auto& robotState =
              IVAR(RobotPose2D<float>, PlanningModule::robotPose2D);
            double smallestDist = 1e3;
            for (int i = 0; i < 2; ++i) {
              double dist = MathsUtils::dist(
                offPositionsInGame[i].x,
                offPositionsInGame[i].y,
                robotState.x,
                robotState.y);
              if (dist < smallestDist) {
                smallestDist = dist;
                target = offPositionsInGame[i];
              }
            }
          }
          if (goToTarget(target, 0.25, 0.25, 0.348888889)) { // tolerance of 50cm, 50 cm, and 20 degrees
            behaviorState = gameplaySequence;
          }
          OVAR(bool, PlanningModule::robotOnSideLine) = false;
        } else {
          auto kickOffTeam = (unsigned) gameData.kickOffTeam;
          auto& ourTeamNumber = IVAR(int, PlanningModule::teamNumber);
          if (kickOffTeam == ourTeamNumber) { // Attack team
            target = startPositionsAtt[OVAR(int, PlanningModule::robocupRole)];
          } else {
            target = startPositionsDef[OVAR(int, PlanningModule::robocupRole)];
          }
          if (goToTarget(target, 0.15, 0.15, 0.174444444)) { // tolerance of 15cm, 15 cm, and 20 degrees
            behaviorState = setSequence;
          }
          OVAR(bool, PlanningModule::robotOnSideLine) = false;
        }
      }
    }
  }
}

void
RobocupSetup::setSequenceAction()
{
  LOG_INFO("Executing RobocupSetup.setSequenceAction()...")
  if (IVAR(PostureState, PlanningModule::postureState) != PostureState::STAND) {
    if (lastMBFinished) {
      auto pConfig = 
        boost::make_shared<MBPostureConfig>(PostureState::STAND, 2.f);
      setupMBRequest(pConfig);
    }
    OVAR(bool, PlanningModule::runVisionModule) = false;
    OVAR(bool, PlanningModule::runLocalizationModule) = false;
  } else {
    if (!IVAR(bool, PlanningModule::whistleDetected)) {
      if (lastSBFinished) {
        auto sConfig = boost::make_shared<SBWhistleDetectorConfig>();
        setupSBRequest(sConfig);
      }
    } else {
      OVAR(bool, PlanningModule::runVisionModule) = true;
      OVAR(bool, PlanningModule::runLocalizationModule) = true;
      OVAR(bool, PlanningModule::localizeWithLastKnown) = true;
      behaviorState = gameplaySequence;
      setSequenceFinished = true;
    }
  }
}

void
RobocupSetup::gameplaySequenceAction()
{
  // Just to check individual behavior set manually.
  auto& role = OVAR(int, PlanningModule::robocupRole);
  role = (int)RobocupRole::GOALKEEPER;
  if (role == (int)RobocupRole::GOALKEEPER) {
    auto planConfig = boost::make_shared<GoalKeeperConfig>();
    setupChildBehaviorRequest(planConfig);
  } else if (role == (int)RobocupRole::DEFENDER) {
    //defenderAction();
  } else if (role == (int)RobocupRole::DEFENSE_SUPPORT) {
    //defSupportAction();
  } else if (role == (int)RobocupRole::OFFENSE_SUPPORT) {
    //offSupportAction();
  } else if (role == (int)RobocupRole::ATTACKER) {
    //attackerAction();
  }
}
