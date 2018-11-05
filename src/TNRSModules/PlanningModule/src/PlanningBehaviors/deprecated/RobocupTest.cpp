/**
 * @file PlanningModule/PlanningBehaviors/RobocupTest.h
 *
 * This file declares the class RobocupTest.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "PlanningModule/include/PlanningBehaviors/RobocupTest.h"

void
RobocupTest::setBehaviorConfig(boost::shared_ptr<BehaviorConfig> behaviorConfig)
{
  this->behaviorConfig =
    (boost::static_pointer_cast < PBRobocupTestConfig > (behaviorConfig));
}

boost::shared_ptr<PBRobocupTestConfig>
RobocupTest::getBehaviorCast()
{
  return boost::static_pointer_cast < PBRobocupTestConfig > (behaviorConfig);
}

void
RobocupTest::initiate()
{
  LOG_INFO("RobocupTest Behavior Started.")
  int cfgType;
  GET_CONFIG( "RobocupTest", (int, behaviorType, cfgType), )
  if (cfgType != -1) {
    getBehaviorCast()->type = (PBRobocupTestTypes) cfgType;
    SET_DVAR(int, forcedBehaviorType, cfgType);
    inBehavior = true;
  } else {
    inBehavior = false;
  }
}

void
RobocupTest::update()
{
  // cout << "IN TEST" << endl;
  if (!inBehavior) return;
  if (!reqStaticBehaviorState() || !reqMotionBehaviorState() || !reqChildBehaviorState()) return;

  updatePostureAndStiffness();

  /*if (!startup) {
   if(lastMBFinished) {
   auto dConfig = boost::make_shared<MBDiveConfig>(MB_DIVE_LONG);
   setupMBRequest(dConfig);
   }
   return;
   }*/

  if (IVAR(bool, PlanningModule::robotFallen)) {
    killallBehaviors();
    if (lastChildFinished) {
      auto planConfig = boost::make_shared<PBGetupSequenceConfig>();
      setupChildBehaviorRequest(planConfig);
    }
    return;
  }

  if (!startup) {
    if (posture == PostureState::STAND_HANDS_BEHIND && stiffness == StiffnessState::MAX) {
      startup = true;
    } else if (stiffness != StiffnessState::MAX) {
      if (lastSBFinished) {
        cout << "Setting stiffness max" << endl;
        auto sConfig =
          boost::make_shared < SBStiffnessConfig > (SBStiffnessTypes::BODY);
        sConfig->sToReach = 1.f;
        setupSBRequest(sConfig);
      }
    } else if (posture != PostureState::STAND_HANDS_BEHIND) {
      if (lastMBFinished) {
        cout << "Setting posture to stand" << endl;
        auto pConfig = boost::make_shared<MBPostureConfig>();
        pConfig->timeToReachP = 2.f;
        pConfig->posture = PostureState::STAND_HANDS_BEHIND;
        setupMBRequest(pConfig);
      }
    }
    return;
  }

  if (getBehaviorCast()->type == PBRobocupTestTypes::TEST_FIELD_EXTRACTION) {
    /*if (!firstGoalReached) {
     if(lastMBFinished) {
     cout << "Setting walk" << endl;
     auto wConfig = boost::make_shared<MBMovementConfig>();
     wConfig->goal = RobotPose2D<float>(3.5, 0.0, 0.0);
     setupMBRequest(wConfig);
     firstGoalReached = true;
     }
     }
     return;*/

    /*if (!set) {
     if(lastChildFinished) {
     auto planConfig = boost::make_shared<PBKickSequenceConfig>(PB_KICK_TO_TARGET);    
     planConfig->align = false;
     planConfig->target = Point2f(4.5f, 0.f); //goal center
     setupChildBehaviorRequest(planConfig);
     }
     set = true;
     }
     return;*/
    /*if(lastMBFinished) {
     auto mConfig = boost::make_shared<MBHeadTrackingConfig>(MB_FOLLOW_BALL);
     setupMBRequest(mConfig);
     }*/
    /*if (!IVAR(bool, PlanningModule::whistleDetected)) {
     if(lastSBFinished) {
     LOG_INFO("Last SB Finished")
     auto sConfig = boost::make_shared<SBWhistleDetectorConfig>();
     setupSBRequest(sConfig);
     }
     
     }
     if (IVAR(bool, PlanningModule::whistleDetected)) {
     LOG_INFO("Whistle Detected")
     }*/
    /*if(lastMBFinished) {
     auto mConfig = boost::make_shared<MBGraspConfig>();
     setupMBRequest(mConfig); 
     }
     return;*/
    /*if (!followBallKilled) {
     if(lastMBFinished) {
     cout << "Setting kick" << endl;
     auto bConfig = boost::make_shared<MBBalanceConfig>();
     bConfig->supportLeg = CHAIN_L_LEG;
     bConfig->type = MB_ZMP;
     setupMBRequest(bConfig);
     followBallKilled = true; 
     }
     }
     return;*/
    /*if (!followBallKilled) {
     if(lastMBFinished) {
     cout << "Setting kick" << endl;
     auto kConfig = boost::make_shared<MBKickConfig>();
     kConfig->type = MB_TIME_OPTIMIZED_KICK;
     kConfig->target = Point2f(4.5, 0.f); //goal center
     setupMBRequest(kConfig);
     followBallKilled = true; 
     }
     }
     return;*/

    //cout << "IN TEST2" << endl;
    /*if (set) {
     if(!lastMBFinished) {
     return;
     } else {
     //inBehavior = false;
     //return;
     set = false;
     }
     }
     //cout << "IN TEST3" << endl;
     if (!IVAR(BallInfo, PlanningModule::ballInfo).found) {
     //cout << "Ball not found." << endl;
     if(lastMBFinished) {
     //cout << "Setting find ball" << endl;
     auto mConfig = boost::make_shared<MBHeadTrackingConfig>(MB_FIND_BALL);
     setupMBRequest(mConfig);
     followBallKilled = false;
     }
     //
     } else {
     //cout << "Ball found." << endl;
     auto ballImage = IVAR(BallInfo, PlanningModule::ballInfo).posImage;
     auto ballRel = IVAR(BallInfo, PlanningModule::ballInfo).posWorld;
     Size imageSize = Size(640, 480);
     if (IVAR(BallInfo, PlanningModule::ballInfo).camera == TOP_CAM)
     imageSize = Size(640, 480);
     else
     imageSize = Size(320, 240);
     //cout << abs(ballImage.x - imageSize.width / 2) << endl;
     //cout << abs(ballImage.y - imageSize.height / 2) << endl;
     
     if (abs(ballImage.x - imageSize.width / 2) < 100 && abs(ballImage.y - imageSize.height / 2) < 100) {
     if (!followBallKilled) {
     lastMotionRequest->kill();
     followBallKilled = true;
     }
     auto ballRel = IVAR(BallInfo, PlanningModule::ballInfo).posWorld;
     float dist = norm(ballRel);
     if (dist > 0.5) {
     auto rP = IVAR(RobotPose2D<float>, PlanningModule::robotPose2D);
     auto targetRel =
     RobotPose2D<float>(ballRel.x - 0.4, ballRel.y + 0.05, 0.0);
     auto ct = cos(rP.theta);
     auto st = sin(rP.theta);
     RobotPose2D<float> target;
     target.x = rP.x + targetRel.x * ct - targetRel.y * st;
     target.y = rP.y + targetRel.x * st + targetRel.y * ct;
     target.theta = rP.theta;
     if(lastMBFinished) {
     //cout << "Setting walk to ball" << endl;
     auto mConfig = boost::make_shared<MBMovementConfig>();
     mConfig->goal = target;
     mConfig->ballTrack = true;
     setupMBRequest(mConfig);
     }
     } else {
     if (!aligned) {
     auto rP = IVAR(RobotPose2D<float>, PlanningModule::robotPose2D);
     auto ballRel = IVAR(BallInfo, PlanningModule::ballInfo).posWorld;
     Point2f ballWorld;
     auto ctRP = cos(rP.theta);
     auto stRP = sin(rP.theta);
     ballWorld.x = rP.x + ballRel.x * ctRP - ballRel.y * stRP;
     ballWorld.y = rP.y + ballRel.x * stRP + ballRel.y * ctRP;
     auto targetWorld = Point2f(2.6, 1.5);
     auto diff = targetWorld - ballWorld;
     auto angle = atan2(diff.y, diff.x);
     auto ct = cos(angle);
     auto st = sin(angle);
     if(lastMBFinished) {
     //cout << "Aligning robot..." << endl;
     //cout << "setting motion config" << endl;
     RobotPose2D<float> target;
     target.x = ballWorld.x + (-0.165 * ct) - (0.075 * st);
     target.y = ballWorld.y + (-0.165 * st) + (0.075 * ct);
     target.theta = angle;
     auto mConfig = boost::make_shared<MBMovementConfig>();
     mConfig->goal = target;
     mConfig->ballTrack = true;
     setupMBRequest(mConfig);
     aligned = true;
     }
     return;
     }
     if(lastMBFinished) {
     //cout << "Setting kick" << endl;
     auto kConfig = boost::make_shared<MBKickConfig>();
     kConfig->target = Point2f(2.6, 1.5); //goal center
     setupMBRequest(kConfig);
     aligned = false;
     set = true;
     }
     }    
     } else {
     if(lastMBFinished) {
     //cout << "Setting follow ball" << endl;
     auto mConfig = boost::make_shared<MBHeadTrackingConfig>(MB_FOLLOW_BALL);
     setupMBRequest(mConfig);
     }
     }
     }
     return;
     }
     if (getBehaviorCast()->type == PBRobocupTestTypes::TEST_LOCALIZATION) {
     //cout << "inlocalization" << endl;
     if (IVAR(bool, PlanningModule::robotLocalized)) {
     if (!firstGoalReached) {
     if(lastChildFinished) {
     auto planConfig = boost::make_shared<PBMoveSequenceConfig>(PB_REACH_TARGET);
     planConfig->target = RobotPose2D<float>(2.0, -2.0, 0.0);
     setupChildBehaviorRequest(planConfig);
     firstGoalReached = true;
     }
     return;
     }
     if (!secondGoalReached) {
     if(lastChildFinished) {
     auto planConfig = boost::make_shared<PBMoveSequenceConfig>(PB_REACH_TARGET);
     planConfig->target = RobotPose2D<float>(2.0, -1.0, M_PI);
     setupChildBehaviorRequest(planConfig);
     secondGoalReached = true;
     }
     return;
     }
     if (!thirdGoalReached) {
     if(lastChildFinished) {
     auto planConfig = boost::make_shared<PBMoveSequenceConfig>(PB_REACH_TARGET);
     planConfig->target = RobotPose2D<float>(0.f, 0.0, 0.0);
     setupChildBehaviorRequest(planConfig);
     thirdGoalReached = true;
     }
     return;
     }
     }*/
  }

  /*if (IVAR(bool, PlanningModule::robotFallen)) {
   if (lastMBStarted)
   lastMotionRequest->kill();
   if(lastChildFinished) {
   auto planConfig = boost::make_shared<PBGetupSequenceConfig>();
   setupChildBehaviorRequest(planConfig);
   }
   }*/

  /*if(lastMBFinished) {
   auto mConfig = boost::make_shared<MBMovementConfig>();
   mConfig->goal = IVAR(RobotPose2D<float>, PlanningModule::robotPose2D);
   mConfig->goal.x += 0;
   mConfig->goal.y += 0;
   mConfig->goal.theta += M_PI / 2;
   setupMBRequest(mConfig);
   }
   return;*/

  /*bool chestButtonPressed = true;//IVAR(vector<float>, PlanningModule::switchSensors)[0] != 0.f;
   if (OVAR(unsigned, PlanningModule::planningState) != PLANNING_STARTUP) {
   if(chestButtonPressed) {
   OVAR(unsigned, PlanningModule::planningState) = PLANNING_STARTUP;
   inBehavior = false;
   return;
   }
   }*/
}

void
RobocupTest::finishBehaviorSafely()
{
  inBehavior = false;
}

void
RobocupTest::loadInitialConfig()
{
  behaviorConfig = boost::make_shared<PBRobocupTestConfig>();
}

/*cout << "Setting Stiffnesses." << endl;
 boost::shared_ptr<MBKickConfig> kConfig =
 boost::make_shared<MBKickConfig>();
 OVAR(BehaviorsRequested, PlanningModule::mBehaviorsRequested).configs.push_back(kConfig);
 OVAR(BehaviorsRequested, PlanningModule::mBehaviorsRequested).id = (pModule->getModuleTime() + 1) * 100;
 mBehaviorSet = true;
 behaviorSetupTime = 0.f;
 return;*/
/*boost::shared_ptr<MBMovementConfig> mConfig =
 boost::make_shared<MBMovementConfig>();
 mConfig->goal = RobotPose2D<float>(1.0, 0.0, 0.0);
 OVAR(BehaviorsRequested, PlanningModule::mBehaviorsRequested).configs.push_back(mConfig);
 OVAR(BehaviorsRequested, PlanningModule::mBehaviorsRequested).id = (pModule->getModuleTime() + 1) * 100;
 mBehaviorSet = true;
 behaviorSetupTime = 0.f;*/
//inBehavior = false;
/*boost::shared_ptr<MBHeadTrackingConfig> htConfig =
 boost::make_shared<MBHeadTrackingConfig>();
 OVAR(BehaviorsRequested, PlanningModule::mBehaviorsRequested).configs.push_back(htConfig);
 OVAR(BehaviorsRequested, PlanningModule::mBehaviorsRequested).id = (pModule->getModuleTime() + 1) * 100;
 mBehaviorSet = true;
 behaviorSetupTime = 0.f;*/
//inBehavior = false;
//cout << "RobocupTest Loop Finish" << endl;
//return;
//unsigned posture = IVAR(unsigned, PlanningModule::postureState);
//cout << "posture:" << posture << endl;
/*if (IVAR(bool, PlanningModule::robotFallen)) {
 if (lastMBStarted)
 lastMotionRequest->kill();
 if(lastChildFinished) {
 auto planConfig = boost::make_shared<PBGetupSequenceConfig>();
 setupChildBehaviorRequest(planConfig);
 }
 }*/
