/**
 * @file MotionModule/GetupModule/Types/KeyFrameMotionGetup.h
 *
 * This file implements the class KeyFrameMotionGetup
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "MotionModule/include/GetupModule/Types/KeyFrameMotionGetup.h"

KFMGetupConfigPtr KeyFrameMotionGetup::getBehaviorCast()
{
  return boost::static_pointer_cast <KFMGetupConfig> (config);
}

void
KeyFrameMotionGetup::initiate()
{
  PRINT("KeyFrameMotionGetup.initiate()")
  motionProxy->post.closeHand("LHand");
  motionProxy->post.closeHand("RHand");
  KeyFrameGetupTypes type = getBehaviorCast()->keyFrameGetupType;
  if (type == KeyFrameGetupTypes::FRONT) {
    endPosture = PostureState::STAND;
    runKeyFrameMotion(getupFromFront);
    inBehavior = true;
  } else if (type == KeyFrameGetupTypes::BACK) {
    endPosture = PostureState::STAND;
    runKeyFrameMotion(getupFromBack);
    inBehavior = true;
  } else if (type == KeyFrameGetupTypes::SIT) {
    endPosture = PostureState::STAND;
    runKeyFrameMotion(getupFromSit);
    inBehavior = true;
  }
}

void
KeyFrameMotionGetup::update()
{
  PRINT("KeyFrameMotionGetup.update()")
  if (runTime > getupTime + cycleTime / 2) {
    OVAR(PostureState, MotionModule::postureState) = endPosture;
    finish();
  }
}

void
KeyFrameMotionGetup::finish()
{
  motionProxy->killAll();
  inBehavior = false;
}
