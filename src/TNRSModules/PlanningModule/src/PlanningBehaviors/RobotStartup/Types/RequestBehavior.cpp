/**
 * @file PlanningBehaviors/RobotStartup/Types/RequestBehavior.cpp
 *
 * This file implements the class RequestBehavior
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "MotionModule/include/MotionConfigs/MBBallThrowConfig.h"
#include "MotionModule/include/MotionConfigs/MBBalanceConfig.h"
#include "MotionModule/include/MotionConfigs/MBHeadControlConfig.h"
#include "MotionModule/include/MotionConfigs/MBMovementConfig.h"
#include "MotionModule/include/MotionConfigs/MBPostureConfig.h"
#include "MotionModule/include/MotionConfigs/MBKickConfig.h"
#include "PlanningModule/include/PlanningRequest.h"
#include "PlanningModule/include/PlanningBehaviors/RobotStartup/Types/RequestBehavior.h"
#include "SBModule/include/SBConfigs.h"

unsigned RequestBehavior::pbRequest;
string RequestBehavior::postureRequest;

RequestBehaviorConfigPtr RequestBehavior::getBehaviorCast()
{
  return boost::static_pointer_cast <RequestBehaviorConfig> (config);
}

void RequestBehavior::initiate()
{
  LOG_INFO("RequestBehavior.initiate()")
  setStartPosture();
  inBehavior = true;
}

void RequestBehavior::update() 
{
  LOG_INFO("RequestBehavior.update()")
  if (requestInProgress()) return;
  updatePostureAndStiffness();

  if (behaviorState == startup) {
    if (setPostureAndStiffness(startPosture, StiffnessState::MAX))
      behaviorState = chestButtonWait;
  } else if (behaviorState == chestButtonWait) {
    //if (IVAR(vector<float>, PlanningModule::switchSensors)[0] != 0.f)
      behaviorState = startRequested;
  } else if (behaviorState == startRequested) {
    auto reqBehaviorId = (PBIds) pbRequest;
    if (reqBehaviorId != PBIds::STARTUP) {
      if (reqBehaviorId == PBIds::ROBOCUP) {
        LOG_INFO("Starting Robocup Gameplay Behavior.")
        //auto robocupConfig = boost::make_shared<RobocupSetupConfig>();
        //PBRequestPtr pbRequest = 
        //  boost::make_shared<RequestPlanningBehavior>(robocupConfig);
        //BaseModule::publishModuleRequest(pbRequest);
        finish();
      } else if (reqBehaviorId == PBIds::KICK_SEQUENCE) {
        LOG_INFO("Starting Kick Sequence Behavior.")
        auto ksConfig = boost::make_shared<FindAndKickConfig>();
        PlanningRequestPtr request = 
          boost::make_shared<RequestPlanningBehavior>(ksConfig);
        BaseModule::publishModuleRequest(request);
        BaseModule::publishModuleRequest(request);
        finish();
      } else if (reqBehaviorId == PBIds::EXTERNAL_INTERFACE) {
        LOG_INFO("Starting NIHA Cognition Behavior.")
        auto urhConfig = boost::make_shared<UserRequestsHandlerConfig>();
        PlanningRequestPtr request = 
          boost::make_shared<RequestPlanningBehavior>(urhConfig);
        BaseModule::publishModuleRequest(request);
        BaseModule::publishModuleRequest(request);
        finish();
      } else {
        // Test case MovementModule
        /*if (!mbInProgress()) {
          LOG_INFO("Setting movement request...")
          auto mConfig = 
            boost::make_shared<MBMovementConfig>(
              RobotPose2D<float>(1.f, 0.f, 0.f),
              false,
              MBMovementTypes::GO_TO_TARGET,
              boost::make_shared<HeadTargetSearchConfig>()
            );
          setupMBRequest(mConfig);
        }*/
        
        // Test case jsoimp kick
        /*if (!mbInProgress()) {
          auto kConfig =
            boost::make_shared <JSOImpKickConfig> (
              Point2f(0.15, -0.0620),
              boost::make_shared<MPComControlConfig>(CHAIN_L_LEG, 1.f)
            );
          //kConfig->target = Point2f(1.15f, 0.95f);
          float reqVel;
          float reqDirection;
          GET_CONFIG("PlanningBehaviors",
            (float, FindAndKick.reqDirection, reqDirection),
            (float, FindAndKick.reqVel, reqVel),
          )
          kConfig->reqVel.x = reqVel * cos (reqDirection * M_PI / 180.0);
          kConfig->reqVel.y = reqVel * sin (reqDirection * M_PI / 180.0);
          setupMBRequest(kConfig);
          inBehavior = false;
        }*/
        //Test case for Zmp control
        if (!mbInProgress()) {
          auto zmpConfig =
            boost::make_shared <ZmpControlConfig> (CHAIN_L_LEG);
          setupMBRequest(zmpConfig);
          finish();
        }

        // Test case ball throw
        /*if (!mbInProgress()) {
          auto bConfig =
            boost::make_shared <MBBallThrowConfig> ();
          setupMBRequest(bConfig);
          inBehavior = false;
        } */
      }
    } else {
      finish();
    }
  }
}

void RequestBehavior::finish()
{
  inBehavior = false;
}

void RequestBehavior::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    GET_CONFIG("PlanningBehaviors",
      (string, RobotStartup.postureRequest, postureRequest), 
      (unsigned, RobotStartup.pbRequest, pbRequest),
    )
    loaded = true;
  }
}

void RequestBehavior::setStartPosture()
{
  try {
    if (postureRequest == "Crouch") {
      startPosture = PostureState::CROUCH;
    } else if (postureRequest == "Sit") {
      startPosture = PostureState::SIT;
    } else if (postureRequest == "StandZero") {
      startPosture = PostureState::STAND_ZERO;
    } else if (postureRequest == "Stand") {
      startPosture = PostureState::STAND;
    } else if (postureRequest == "StandHandsBehind") {
      startPosture = PostureState::STAND_HANDS_BEHIND;
    } else {
      throw 
      BehaviorException(
        this,
        "Invalid start posture requested. See Config: PlanningBehavior.ini.",
        true,
        EXC_INVALID_BEHAVIOR
      );
    } 
  } catch (BehaviorException& e) {
    cout << e.what();
    LOG_INFO("Setting default posture: PostureState::CROUCH.");
    startPosture = PostureState::CROUCH;
  }
}