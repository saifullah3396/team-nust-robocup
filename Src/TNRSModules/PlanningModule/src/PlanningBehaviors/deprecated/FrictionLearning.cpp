/**
 * @file PlanningModule/PlanningBehaviors/FrictionLearning.h
 *
 * This file declares the class FrictionLearning.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "PlanningModule/include/PlanningBehaviors/FrictionLearning.h"

void
FrictionLearning::setBehaviorConfig(
  boost::shared_ptr<BehaviorConfig> behaviorConfig)
{
  this->behaviorConfig =
    (boost::static_pointer_cast < PBFrictionLearningConfig > (behaviorConfig));
}

boost::shared_ptr<PBFrictionLearningConfig>
FrictionLearning::getBehaviorCast()
{
  return boost::static_pointer_cast < PBFrictionLearningConfig > (behaviorConfig);
}

void
FrictionLearning::finishBehaviorSafely()
{
  inBehavior = false;
}

void
FrictionLearning::loadInitialConfig()
{
  behaviorConfig = boost::make_shared<PBFrictionLearningConfig>();
}

void
FrictionLearning::initiate()
{
  PRINT("FrictionLearning behavior initiated...")
  //Dont start vision module
  //OVAR(bool, PlanningModule::runVisionModule) = true;
  inBehavior = true;
}

void
FrictionLearning::update()
{
  PRINT("Executing FrictionLearning.update()...")
  if (!inBehavior) return;
  updatePostureAndStiffness();
  if (!reqStaticBehaviorState() || !reqMotionBehaviorState() || !reqChildBehaviorState()) return;

  if (behaviorState == startup) {
    startupAction();
  } else if (behaviorState == waitForHeadTap) {
    waitForHeadTapAction();
  } else if (behaviorState == searchBall) {
    searchBallAction();
  } else if (behaviorState == alignToKick) {
    alignToKickAction();
  } else if (behaviorState == kickBall) {
    kickBallAction();
  }
}

void
FrictionLearning::startupAction()
{
  PRINT("Executing FrictionLearning.startupAction()...")
  behaviorState = kickBall;
  return;
  if (posture == PostureState::STAND && stiffness == StiffnessState::ROBOCUP) {
    OVAR(bool, PlanningModule::runVisionModule) = true;
    //OVAR(bool, PlanningModule::runLocalizationModule) = true;
    behaviorState = waitForHeadTap;
  } else if (stiffness != StiffnessState::ROBOCUP) {
    if (lastSBFinished) {
      PRINT("Setting stiffness high at startup.")
      auto sConfig =
        boost::make_shared < SBStiffnessConfig > (SBStiffnessTypes::ROBOCUP);
      setupSBRequest(sConfig);
    }
  } else if (posture != PostureState::STAND) {
    if (lastMBFinished) {
      PRINT("Setting posture to stand at startup.")
      auto pConfig = boost::make_shared<MBPostureConfig>();
      pConfig->timeToReachP = 2.f;
      pConfig->posture = PostureState::STAND;
      setupMBRequest(pConfig);
    }
  }
}

void
FrictionLearning::waitForHeadTapAction()
{
  bool headTapped =
    IVAR(vector<float>, PlanningModule::touchSensors)[HEAD_TOUCH_MIDDLE] > 0.f;
  if (true) { //if(headTapped) {
    behaviorState = searchBall;
  }
}

bool
FrictionLearning::shouldSearchBall()
{
  auto& bInfo = IVAR(BallInfo, PlanningModule::ballInfo);
  if (!bInfo.found) return true;
  return false;
}

void
FrictionLearning::searchBallAction()
{
  //PRINT("Exeucting FrictionLearning.searchBallAction()")
  if (shouldSearchBall()) {
    if (lastMBFinished) {
      //cout << "Setting find ball" << endl;
      auto mConfig =
        boost::make_shared < MBHeadTrackingConfig > (MBHeadTrackingTypes::FIND_BALL);
      setupMBRequest(mConfig);
    }
  } else {
    //auto& bInfo = IVAR(BallInfo, PlanningModule::ballInfo);
    //cout << "velMag: " << tbInfo.velWorld << endl;
    //cout << "velRel: " << bInfo.velRel << endl;
    //cout << "final" << endl;
    behaviorState = alignToKick;
  }
}

void
FrictionLearning::findBestBallAlignment(RobotPose2D<float>& alignPosition)
{
  bool useLeftFoot = false;
  auto& bInfo = IVAR(BallInfo, PlanningModule::ballInfo);
  auto ballRel = bInfo.posRel;
  auto angle = atan2(lastReqVelocity.y, lastReqVelocity.x);
  if (angle < 0) {
    useLeftFoot = true;
  } else {
    useLeftFoot = false;
  }
  if (fabsf(angle) < 30 * M_PI / 180) {
    alignPosition.x = ballRel.x + -0.165;
    alignPosition.theta = 0.f;
    if (!useLeftFoot) { // Right foot in front of ball
      alignPosition.y = ballRel.y + 0.05;
    } else { // Left foot in front of ball
      alignPosition.y = ballRel.y - 0.05;
    }
  } else {
    alignPosition.x = ballRel.x + -0.165;
    alignPosition.y = ballRel.y;
    alignPosition.theta = 0.f;
  }
}

void
FrictionLearning::alignToKickAction()
{
  // Align to pass the ball to best target
  RobotPose2D<float> target;
  findBestBallAlignment(target);
  if (lastMBFinished) {
    PRINT("Setting movement request...")
    auto mConfig = boost::make_shared<MBMovementConfig>();
    mConfig->goal = target;
    mConfig->ballTrack = true;
    setupMBRequest(mConfig);
    behaviorState = kickBall;
  }
}

void
FrictionLearning::kickBallAction()
{
  Point2f ball(0.165, -0.05);
  auto kConfig =
    boost::make_shared <MBKickConfig> (MBKickTypes::FIXED_VELOCITY, ball);
  float mag = 0.5;
  kConfig->reqVel.x = 0.5 * cos(0);
  kConfig->reqVel.y = 0.5 * sin(0);
  setupMBRequest(kConfig);
  inBehavior = false;
  /*auto ballRel = IVAR(BallInfo, PlanningModule::ballInfo).posRel;
  if (ballRel.x < 0.160 || ballRel.x > 0.170 || ballRel.y > -0.045 || ballRel.y < -0.055) {
    behaviorState = alignToKick;
  } else {
    cout << "Should kick now." << endl;
    if(lastMBFinished) {
     auto kConfig = 
     boost::make_shared<MBKickConfig>(MBKickTypes::FIXED_VELOCITY);
     kConfig->ball = ballRel;
     kConfig->reqVel = lastReqVelocity;
     kConfig->ballTrack = true;
     setupMBRequest(kConfig);
     behaviorState = startup;
     }
  }*/
}
