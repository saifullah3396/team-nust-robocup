/**
 * @file PlanningBehaviors/RobotStartup/Types/RequestBehavior.cpp
 *
 * This file implements the class RequestBehavior
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "MotionModule/include/MotionConfigs/MBPostureConfig.h"
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
  PRINT("RequestBehavior.initiate()")
  setStartPosture();
  inBehavior = true;
}

void RequestBehavior::update() 
{
  if (requestInProgress()) return;
  updatePostureAndStiffness();

  if (!started) {
    if (posture == startPosture && stiffness == StiffnessState::ROBOCUP) {
      started = true;
    } else if (stiffness != StiffnessState::ROBOCUP) {
      if (!sbInProgress()) {
        PRINT("RequestBehavior: Setting stiffness high at startup.")
        auto sConfig =
          boost::make_shared <SBStiffnessConfig> (StiffnessState::ROBOCUP);
        setupSBRequest(sConfig);
      }
    } else if (posture != startPosture) {
      if (!mbInProgress()) {
        PRINT("RequestBehavior: Setting posture to requested posture at startup.")
        auto pConfig = 
          boost::make_shared<MBPostureConfig>(startPosture);
        setupMBRequest(pConfig);
      }
    }
    return;
  }

  bool chestButtonPressed = true; //IVAR(vector<float>, PlanningModule::switchSensors)[0] != 0.f;
  if (chestButtonPressed) {
    auto reqBehaviorId = (PBIds) pbRequest;
    if (reqBehaviorId != PBIds::STARTUP) {
      if (reqBehaviorId == PBIds::ROBOCUP) {
        PRINT("Starting Robocup Gameplay Behavior.")
        //auto robocupConfig = boost::make_shared<RobocupSetupConfig>();
        //PBRequestPtr pbRequest = 
        //  boost::make_shared<RequestPlanningBehavior>(robocupConfig);
        //BaseModule::publishModuleRequest(pbRequest);
        finish();
      } else if (reqBehaviorId == PBIds::KICK_SEQUENCE) {
        PRINT("Starting Kick Sequence Behavior.")
        auto ksConfig = boost::make_shared<BallInterceptConfig>();
        PlanningRequestPtr request = 
          boost::make_shared<RequestPlanningBehavior>(ksConfig);
        BaseModule::publishModuleRequest(request);
        BaseModule::publishModuleRequest(request);
        finish();
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
  GET_CONFIG(
    "PlanningBehaviors",
    (string, RobotStartup.postureRequest, postureRequest), 
    (unsigned, RobotStartup.pbRequest, pbRequest),
  )
}

void RequestBehavior::setStartPosture() throw (BehaviorException)
{
  try {
    if (postureRequest == "Crouch") {
      startPosture = PostureState::CROUCH;
    } else if (postureRequest == "Sit") {
      startPosture = PostureState::SIT;
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
    PRINT("Setting default posture: PostureState::CROUCH.");
    startPosture = PostureState::CROUCH;
  }
}
