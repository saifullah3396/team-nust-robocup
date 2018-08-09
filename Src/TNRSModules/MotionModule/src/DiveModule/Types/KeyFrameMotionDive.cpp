/**
 * @file MotionModule/DiveModule/Types/KeyFrameMotionDive.h
 *
 * This file implements the class KeyFrameMotionDive
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "MotionModule/include/DiveModule/Types/KeyFrameMotionDive.h"

KFMDiveConfigPtr KeyFrameMotionDive::getBehaviorCast()
{
  return boost::static_pointer_cast<KFMDiveConfig> (config);
}

void
KeyFrameMotionDive::initiate()
{
  //PRINT("KeyFrameMotionDive.initiate()")
  motionProxy->post.closeHand("LHand");
  motionProxy->post.closeHand("RHand");
  KeyFrameDiveTypes type = getBehaviorCast()->keyFrameDiveType;
  if (type == KeyFrameDiveTypes::IN_PLACE) {
    endPosture = PostureState::DIVE_IN_PLACE;
    runKeyFrameMotion(diveInPlace);
    inBehavior = true;
  } else if (type == KeyFrameDiveTypes::SUMO) {
    endPosture = PostureState::DIVE_SUMO;
    runKeyFrameMotion(diveSumo);
    inBehavior = true;
  } else if (type == KeyFrameDiveTypes::LEFT) {
    endPosture = PostureState::DIVE_LEFT;
    runKeyFrameMotion(diveLeft);
    inBehavior = true;
  } else if (type == KeyFrameDiveTypes::RIGHT) {
    endPosture = PostureState::DIVE_RIGHT;
    runKeyFrameMotion(diveRight);
    inBehavior = true;
  }
}

void
KeyFrameMotionDive::update()
{
  //PRINT("KeyFrameMotionDive.update()")
  if (runTime > diveTime + cycleTime / 2) {
    OVAR(PostureState, MotionModule::postureState) = endPosture;
    finish();
  }
}

void
KeyFrameMotionDive::finish()
{
  motionProxy->killAll();
  inBehavior = false;
}
