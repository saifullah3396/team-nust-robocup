/**
 * @file PlanningModule/PlanningBehavior.cpp
 *
 * This file implements the base class for all types of planning behaviors.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#include "PlanningModule/include/PlanningBehavior.h"
#include "SBModule/include/SBRequest.h"
#include "MotionModule/include/MotionRequest.h"
//#include "MotionModule/include/MotionConfigs/MBMovementConfig.h"
#include "MotionModule/include/MotionConfigs/MBPostureConfig.h"
#include "SBModule/include/SBConfigs.h"

void PlanningBehavior::updatePostureAndStiffness()
{
  posture = IVAR(PostureState, PlanningModule::postureState);
  stiffness = IVAR(StiffnessState, PlanningModule::stiffnessState);
}

bool PlanningBehavior::setPostureAndStiffness(
  const PostureState& desPosture, 
  const StiffnessState& desStiffness, 
  const float& postureTime)
{ 
  cout << "int set posture and stiffness" << endl;
  if (posture == desPosture && 
      stiffness == desStiffness) 
  {
    cout << "true" << endl;
    return true;
  } else if (stiffness != desStiffness) {
    cout << "tryig to set stiffness" << endl;
    if (!sbInProgress()) {
      auto sConfig =
        boost::make_shared <SBStiffnessConfig> (
          desStiffness);
      setupSBRequest(sConfig);
    }
    return false;
  } else if (posture != desPosture) {
    cout << "tryig to set posture" << endl;
    if (!mbInProgress()) {
      auto pConfig = 
        boost::make_shared<MBPostureConfig>(
          desPosture, postureTime);
      setupMBRequest(pConfig);
    }
    return false;
  }
}

void PlanningBehavior::killStaticBehavior() {
  auto request = boost::make_shared<KillStaticBehavior>();
  BaseModule::publishModuleRequest(request);
}

void PlanningBehavior::killMotionBehavior() {
  auto request = boost::make_shared<KillMotionBehavior>();
  BaseModule::publishModuleRequest(request);
}
/*
bool
PlanningBehavior::goToTarget(const RobotPose2D<float>& target,
  const float& positionTolX, const float& positionTolY,
  const float& positionTolTheta, const bool& trackBall)
{
  auto& robotPose = IVAR(RobotPose2D<float>, PlanningModule::robotPose2D);
  if (abs(robotPose.x - target.x) < positionTolX && abs(robotPose.y - target.y) < positionTolY && abs(
    robotPose.theta - target.theta) < positionTolTheta) {
    PRINT("Already at position...")
    return true;
  } else {
    if (!lastMBFinished && lastMotionRequest->id != (unsigned) MBIds::MOVEMENT) {
      return false;
    }
    //if (lastMBFinished) {
    // lastMBFinished is not used in walk since walk uses reinitiate 
    // function to reassign the behaviorConfig if it is changed
    PRINT("Setting movement request...")
    auto mConfig = boost::make_shared<MBMovementConfig>();
    mConfig->goal = target;
    mConfig->ballTrack = trackBall;
    setupMBRequest(mConfig);
    //}
    return false;
  }
}
*//*
bool
PlanningBehavior::penalisedRobot()
{
  if (!waitForUnpenalise) {
    if (!penaliseMotion) {
      auto& playerNumber = IVAR(int, PlanningModule::playerNumber);
      auto& teamNumber = IVAR(int, PlanningModule::teamNumber);
      auto& gameData = IVAR(RoboCupGameControlData, PlanningModule::gameData);
      TeamInfo& team = gameData.teams[
        gameData.teams[0].teamNumber == teamNumber ? 0 : 1];
      if ((unsigned) team.players[playerNumber].penalty == 0) {
        penaliseMotion = true;
        killallBehaviors();
        return true;
      } else {
        return false;
      }
    } else {
      if (posture == PostureState::STAND && stiffness == StiffnessState::ROBOCUP) {
        penaliseMotion = false;
        waitForUnpenalise = true;
      } else if (stiffness != StiffnessState::ROBOCUP) {
        if (lastSBFinished) {
          PRINT("Setting stiffness high at startup.")
          auto sConfig =
            boost::make_shared < SBStiffnessConfig > (StiffnessState::ROBOCUP);
          setupSBRequest(sConfig);
        }
      } else if (posture != PostureState::STAND) {
        if (lastMBFinished) {
          PRINT("Setting posture to sit at startup.")
          auto pConfig = boost::make_shared<MBPostureConfig>(PostureState::STAND);
          setupMBRequest(pConfig);
        }
      }
      return true;
    }
  } else {
    auto& playerNumber = IVAR(int, PlanningModule::playerNumber);
    auto& teamNumber = IVAR(int, PlanningModule::teamNumber);
    auto& gameData = IVAR(RoboCupGameControlData, PlanningModule::gameData);
    auto& team = gameData.teams[
      gameData.teams[0].teamNumber == teamNumber ? 0 : 1];
    if ((unsigned) team.players[playerNumber].penalty == 0) {
      waitForUnpenalise = false;
      return false;
    } else {
      return true;
    }
  }*/
  /*if(!robotPenalised) {
   auto gameData = IVAR(RoboCupGameControlData, PlanningModule::gameData);
   if((unsigned)gameData.robotInfo.penalty > 0) {
   robotPenalised = true;
   timeTillUnpenalised = (float)gameData.robotInfo.secsTillUnpenalised;
   //timePenalised = pModule->getModuleTime();
   }
   } else {
   if (pModule->getModuleTime() - timePenalised > timeTillUnpenalised)
   }*/
//}

void PlanningBehavior::setupSBRequest(const SBConfigPtr& config)
{
  auto rsb = boost::make_shared<RequestStaticBehavior>(config);
  lastStaticRequest = rsb;
  BaseModule::publishModuleRequest(rsb);
  sRequestTime = pModule->getModuleTime();
}

void PlanningBehavior::setupMBRequest(const MBConfigPtr& config)
{
  auto rmb = boost::make_shared<RequestMotionBehavior>(config);
  lastMotionRequest = rmb;
  BaseModule::publishModuleRequest(rmb);
  mRequestTime = pModule->getModuleTime();
}

bool PlanningBehavior::sbInProgress()
{
  return behaviorInProgress(
    IVAR(BehaviorInfo, PlanningModule::sBehaviorInfo));
}

bool PlanningBehavior::mbInProgress()
{
  return behaviorInProgress(
    IVAR(BehaviorInfo, PlanningModule::mBehaviorInfo));
}

bool PlanningBehavior::behaviorInProgress(const BehaviorInfo& info)
{
  if(info.isInitiated())
  {
    if (info.isRunning()) 
      return true;
  }
  return false;
}

bool PlanningBehavior::requestInProgress()
{
  bool inProgress = false;
  auto& sbInfo = IVAR(BehaviorInfo, PlanningModule::sBehaviorInfo);
  if(requestInProgress(lastStaticRequest, sbRunning, sbInfo, sRequestTime))
    inProgress = true;
  auto& mbInfo = IVAR(BehaviorInfo, PlanningModule::mBehaviorInfo);
  if(requestInProgress(lastMotionRequest, mbRunning, mbInfo, mRequestTime))
    inProgress = true;
  return inProgress;
}

bool PlanningBehavior::requestInProgress(
  BehaviorRequestPtr& req,
  BehaviorConfigPtr& acceptedBehavior,
  const BehaviorInfo& feedback,
  const float& requestStartTime)
{
  if (!req)
    return false;
    
  if (
    pModule->getModuleTime() - requestStartTime > 
    maxRequestTimeout) 
  {
    req.reset();
    return false;
  }
    
  if (req->isReceived()) {
    //! Request received on the other end
    if (req->getAccepted()) {
      //cout << "Accepted request" << endl;
      if (feedback.isInitiated() && 
          req->getReqConfig()->id == feedback.getConfig()->id) 
      {
        //cout << "Behavior initiated on request" << endl;
        acceptedBehavior = feedback.getConfig();
        req.reset();
        return false;
      }
      return true;
    } else {
      acceptedBehavior.reset();
      req.reset();
    }
    return false;
  }
  return true;
}
/*
bool
PlanningBehavior::reqRelocalization()
{
  if (OVAR(bool, PlanningModule::resetLocalizer)) {
    auto& resetted = IVAR(bool, PlanningModule::localizerResetted);
    if (resetted) {
      OVAR(bool, PlanningModule::resetLocalizer) = false;
      return false;
    }
    return true;
  } else {
    OVAR(bool, PlanningModule::resetLocalizer) = true;
    return true;
  }
}
*/
