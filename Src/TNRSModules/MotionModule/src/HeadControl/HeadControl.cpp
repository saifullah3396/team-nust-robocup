/**
 * @file MotionModule/src/HeadControl/HeadControl.cpp
 *
 * This file implements the class HeadControl
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 20 Nov 2017
 */

#include "MotionModule/include/HeadControl/HeadControl.h"
#include "MotionModule/include/HeadControl/Types/HeadTargetTrack.h"
#include "MotionModule/include/HeadControl/Types/HeadTargetSearch.h"

boost::shared_ptr<HeadControl> HeadControl::getType(
  MotionModule* motionModule, const BehaviorConfigPtr& cfg) 
{ 
  HeadControl* hc;
  switch (cfg->type) {
      case (unsigned) MBHeadControlTypes::HEAD_TARGET_TRACK: 
        hc = new HeadTargetTrack(motionModule, cfg); break;
      case (unsigned) MBHeadControlTypes::HEAD_TARGET_SEARCH: 
        hc = new HeadTargetSearch(motionModule, cfg); break;
      default: hc = new HeadTargetTrack(motionModule, cfg); break;
  }
  return boost::shared_ptr<HeadControl>(hc);
}

bool HeadControl::findTarget(
  const HeadTargetTypes& targetType, 
  Point2f& targetXY, 
  float& targetZ) throw (BehaviorException)
{
  try {
    if (targetType == HeadTargetTypes::BALL) {
      auto& ballInfo = IVAR(BallInfo, MotionModule::ballInfo);
      if (ballInfo.found) {
        targetXY = ballInfo.posRel;
        targetZ = 0.05f;
        return true;
      } else {
        return false;
      }
    } else if (targetType == HeadTargetTypes::GOAL) {
      auto& goalInfo = IVAR(GoalInfo, MotionModule::goalInfo);
      if (
        goalInfo.found && 
        goalInfo.leftPost.x > -50 && 
        goalInfo.rightPost.x > -50) 
      {
        targetXY = goalInfo.mid;
        targetZ = 0.f;
        return true;
      } else {
        return false;
      }
    } else if (targetType == HeadTargetTypes::LANDMARKS) {
      if (IVAR(bool, MotionModule::landmarksFound)) {
        targetZ = -1.f;
        return true;
      } else {
        return false;
      }
    } else {
      throw BehaviorException(this, "Undefined target type for HeadTargetSearch", false, EXC_INVALID_BEHAVIOR_SETUP);
    }
  } catch (BehaviorException& e) {
    cout << e.what();
    inBehavior = false;
    return false;
  }
}
