/**
 * @file PlanningBehaviors/KickSequence/Types/FindAndKick.cpp
 *
 * This file implements the class FindAndKick.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "Utils/include/ConfigMacros.h"
#include "MotionModule/include/MotionConfigs/MBPostureConfig.h"
#include "MotionModule/include/MotionConfigs/MBBalanceConfig.h"
#include "MotionModule/include/MotionConfigs/MBKickConfig.h"
#include "MotionModule/include/MotionConfigs/MBHeadControlConfig.h"
#include "SBModule/include/SBConfigs.h"
#include "PlanningModule/include/PlanningBehaviors/KickSequence/Types/FindAndKick.h"
#include "Utils/include/Solvers/MotionEquationSolver.h"
#include "VisionModule/include/VisionRequest.h"
#include "Utils/include/VisionUtils.h"
#include <opencv2/core/eigen.hpp>

#ifdef SIMULATION
float FindAndKick::coeffDamping;
#else
float FindAndKick::rollingFriction;
#endif

FindAndKickConfigPtr FindAndKick::getBehaviorCast()
{
  return boost::static_pointer_cast <FindAndKickConfig> (config);
}

void FindAndKick::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    #ifdef SIMULATION
    GET_CONFIG("EnvProperties",
      (float, coeffDamping, coeffDamping),
    )
    #else
    GET_CONFIG("EnvProperties",
      (float, coeffRF, rollingFriction),
    )
    #endif
    float reqVel;
    float reqDirection;
    GET_CONFIG("PlanningBehaviors",
      (float, FindAndKick.reqDirection, reqDirection),
      (float, FindAndKick.reqVel, reqVel),
    )
    desKickVel.x = reqVel * cos (reqDirection * M_PI / 180.0);
    desKickVel.y = reqVel * sin (reqDirection * M_PI / 180.0);    
    loaded = true;
  }
}

void
FindAndKick::initiate()
{
  LOG_INFO("FindAndKick.initiate()...")
  inBehavior = true;
}

void FindAndKick::update()
{
  //LOG_INFO("FindAndKick.update()...")
	if (requestInProgress()) return;
  updatePostureAndStiffness();
  auto vRequest = boost::make_shared<SwitchVision>(true);
  BaseModule::publishModuleRequest(vRequest);
  if (!mbInProgress()) {
    //cout << "Setting find ball" << endl;
    auto mConfig =
      boost::make_shared <HeadTargetSearchConfig> (
        HeadTargetTypes::BALL,
        true,
        0.5f,
        0.f * M_PI / 180,
        30.f * M_PI / 180
      );
    setupMBRequest(mConfig);
  }
  return;
  /*if (behaviorState == headTapWait) {
    headTapWaitAction();
  } else if (behaviorState == startup) {
    startupAction();
  } else if (behaviorState == searchBall) {
    searchBallAction();
  } else if (behaviorState == kick) {
    kickAction();
  }*/
}

void FindAndKick::finish()
{
  inBehavior = false;
}

void FindAndKick::headTapWaitAction()
{
  LOG_INFO("FindAndKick.headTapWaitAction()...")
  if (setPostureAndStiffness(PostureState::CROUCH, StiffnessState::ROBOCUP)) {
    bool headTapped =
      IVAR(vector<float>, PlanningModule::touchSensors)[HEAD_TOUCH_MIDDLE] > 0.f;
    if(headTapped) {
      behaviorState = startup;
    }
  }
}

void FindAndKick::startupAction()
{
  LOG_INFO("FindAndKick.startupAction()...")
  if(setPostureAndStiffness(PostureState::STAND, StiffnessState::ROBOCUP)) {
    #ifndef SIMULATION
    auto vRequest = boost::make_shared<SwitchVision>(true);
    BaseModule::publishModuleRequest(vRequest);
    #endif
    behaviorState = searchBall; 
  }
}

void FindAndKick::searchBallAction()
{
  //LOG_INFO("FindAndKick.searchBallAction()...")
  bool headTapped =
    IVAR(vector<float>, PlanningModule::touchSensors)[HEAD_TOUCH_MIDDLE] > 0.f;
  if(headTapped) {
    killMotionBehavior();
    behaviorState = headTapWait;
  }
  auto& bInfo = IVAR(BallInfo, PlanningModule::ballInfo);
  if (!bInfo.found) {
    if (!mbInProgress()) {
      //cout << "Setting find ball" << endl;
      auto mConfig =
        boost::make_shared <HeadTargetSearchConfig> (
          HeadTargetTypes::BALL,
          true,
          0.5f,
          0.f * M_PI / 180,
          30.f * M_PI / 180
        );
      setupMBRequest(mConfig);
    }
  } else {
    if (
      bInfo.posRel.x < 0.16f && // 16cm
      fabsf(bInfo.posRel.y) < 0.05f) // 5 cm
    {  
      behaviorState = kick;
    }
  }
}

void FindAndKick::kickAction()
{
  LOG_INFO("FindAndKick.kickAction()...")
  auto& bInfo = IVAR(BallInfo, PlanningModule::ballInfo);
  if (!mbInProgress()) {
    auto kConfig =
      boost::make_shared <JSOImpKickConfig> (
        bInfo.posRel,
        boost::make_shared<MPComControlConfig>(CHAIN_L_LEG, 1.f)
      );
    kConfig->reqVel = desKickVel;
    setupMBRequest(kConfig);
    behaviorState = headTapWait;
    auto vRequest = boost::make_shared<SwitchVision>(false);
    BaseModule::publishModuleRequest(vRequest);
  }
}
