/**
 * @file PlanningModule/PlanningBehaviors/MoveSequence.h
 *
 * This file declares the class MoveSequence.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "PlanningModule/include/PlanningBehaviors/MoveSequence.h"

void
MoveSequence::setBehaviorConfig(
  boost::shared_ptr<BehaviorConfig> behaviorConfig)
{
  this->behaviorConfig =
    (boost::static_pointer_cast < PBMoveSequenceConfig > (behaviorConfig));
}

boost::shared_ptr<PBMoveSequenceConfig>
MoveSequence::getBehaviorCast()
{
  return boost::static_pointer_cast < PBMoveSequenceConfig > (behaviorConfig);
}

void
MoveSequence::initiate()
{
  auto& type = getBehaviorCast()->type;
  if (type == PBMoveSequenceTypes::REACH_TARGET) {
    if (getBehaviorCast()->target.x == -100) {
      // cout << "target.x: " << getBehaviorCast()->target.x << endl;
      // cout << "target.y: " << getBehaviorCast()->target.y << endl;
      // cout << "target.theta: " << getBehaviorCast()->target.theta << endl;
      inBehavior = false;
      return;
    }
    inBehavior = true;
    return;
  } else if (type == PBMoveSequenceTypes::REACH_BALL) {
    inBehavior = true;
    return;
  } else if (type == PBMoveSequenceTypes::DRIBBLE_BALL) {
    inBehavior = true;
    return;
  } else {
    inBehavior = false;
    return;
  }
}

void
MoveSequence::update()
{
  if (!inBehavior) return;
  cout << "In Move Sequence" << endl;
  if (!reqStaticBehaviorState() || !reqMotionBehaviorState() || !reqChildBehaviorState()) return;

  /* if (IVAR(bool, PlanningModule::robotFallen)) {
   if (lastMBStarted)
   lastMotionRequest->kill();
   if(lastChildFinished) {
   auto planConfig = boost::make_shared<PBGetUpSequenceConfig>();
   setupChildBehaviorRequest(planConfig);
   }
   }*/

  auto& type = getBehaviorCast()->type;
  if (type == PBMoveSequenceTypes::REACH_TARGET) {
    auto target = getBehaviorCast()->target;
    auto rP = IVAR(RobotPose2D<float>, PlanningModule::robotPose2D);
    float dist = MathsUtils::dist(target.x, target.y, rP.x, rP.y);
    cout << "target.x: " << target.x << endl;
    cout << "target.y: " << target.y << endl;
    cout << "target.theta: " << target.theta << endl;
    if (dist < 0.20) { //! 20 cm
      float angleDiff = MathsUtils::diffAngle(target.theta, rP.theta);
      if (abs(angleDiff) < 0.26180) { //! 15 degrees
        if (lastMBFinished) {
          //   cout << "child finished" << endl;
          inBehavior = false;
          return;
        }
      } else {
        if (lastMBFinished) {
          //target.x = 0.0;
          //target.y = 0.0;
          auto mConfig = boost::make_shared<MBMovementConfig>();
          mConfig->goal = target;
          setupMBRequest(mConfig);
        }
      }
    } else {
      if (lastMBFinished) {
        //target.theta = IVAR(RobotPose2D<float>, PlanningModule::robotPose2D).theta;
        auto mConfig = boost::make_shared<MBMovementConfig>();
        mConfig->goal = target;
        setupMBRequest(mConfig);
      }
    }
    return;
  } else if (type == PBMoveSequenceTypes::REACH_BALL) {
    if (walkDone) {
      if (lastMBFinished) {
        inBehavior = false;
        return;
      }
    } else {
      if (!IVAR(BallInfo, PlanningModule::ballInfo).found) {
        inBehavior = false;
        return;
      }
    }
    /*if (!walkDone) {
     //if (!IVAR(BallInfo, PlanningModule::ballInfo).found)
     //{
     //  inBehavior = false;
     //  return;
     //}*/
    auto rP = IVAR(RobotPose2D<float>, PlanningModule::robotPose2D);
    auto ballRel = IVAR(BallInfo, PlanningModule::ballInfo).posRel;
    cout << "BallRel in Walk" << ballRel << endl;
    auto targetRel = RobotPose2D<float>(ballRel.x - 0.25, ballRel.y, 0.0);
    auto ct = cos(rP.theta);
    auto st = sin(rP.theta);
    RobotPose2D<float> target;
    target.x = rP.x + targetRel.x * ct - targetRel.y * st;
    target.y = rP.y + targetRel.x * st + targetRel.y * ct;
    target.theta = rP.theta;
    if (lastMBFinished) {
      auto mConfig = boost::make_shared<MBMovementConfig>();
      mConfig->goal = target;
      mConfig->ballTrack = true;
      setupMBRequest(mConfig);
      walkDone = true;
    }
    float dist = MathsUtils::dist(target.x, target.y, rP.x, rP.y);
    if (dist < 0.1) { //! 20 cm
      float angleDiff = MathsUtils::diffAngle(target.theta, rP.theta);
      if (abs(angleDiff) < 0.26180) { //! 15 degrees
        if (lastMBFinished) {
          inBehavior = false;
          return;
          //auto mConfig = boost::make_shared<MBHeadTrackingConfig>(MB_FIND_BALL_NEAR);
          //setupMBRequest(mConfig);
          //walkDone = true;
          //return;
        }
      } else {
        if (lastMBFinished) {
          //target.x = 0.0;
          //target.y = 0.0;
          auto mConfig = boost::make_shared<MBMovementConfig>();
          mConfig->goal = target;
          mConfig->ballTrack = true;
          setupMBRequest(mConfig);
        }
      }
    } else {
      if (lastMBFinished) {
        //target.theta = IVAR(RobotPose2D<float>, PlanningModule::robotPose2D).theta;
        auto mConfig = boost::make_shared<MBMovementConfig>();
        mConfig->goal = target;
        mConfig->ballTrack = true;
        setupMBRequest(mConfig);
      }
    }
  } else if (type == PBMoveSequenceTypes::DRIBBLE_BALL) {
    if (!IVAR(BallInfo, PlanningModule::ballInfo).found) return;
    auto rP = IVAR(RobotPose2D<float>, PlanningModule::robotPose2D);
    auto ballRel = IVAR(BallInfo, PlanningModule::ballInfo).posRel;
    float dist = norm(ballRel);
    if (dist > 0.20) {
      inBehavior = false;
      return;
    }
    auto targetRel = RobotPose2D<float>(ballRel.x + 0.15, ballRel.y, 0.0);
    auto ct = cos(rP.theta);
    auto st = sin(rP.theta);
    RobotPose2D<float> target;
    target.x = rP.x + targetRel.x * ct - targetRel.y * st;
    target.y = rP.y + targetRel.x * st + targetRel.y * ct;
    target.theta = rP.theta;
    auto mConfig = boost::make_shared<MBMovementConfig>();
    mConfig->goal = target;
    mConfig->ballTrack = true;
    setupMBRequest(mConfig);
    inBehavior = false;
    return;
  }
}

void
MoveSequence::finishBehaviorSafely()
{
  inBehavior = false;
}

void
MoveSequence::loadInitialConfig()
{
  behaviorConfig = boost::make_shared<PBMoveSequenceConfig>();
}
