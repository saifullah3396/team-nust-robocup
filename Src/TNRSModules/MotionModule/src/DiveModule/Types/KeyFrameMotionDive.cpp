/**
 * @file MotionModule/DiveModule/Types/KeyFrameMotionDive.h
 *
 * This file implements the class KeyFrameMotionDive
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "MotionModule/include/DiveModule/Types/KeyFrameMotionDive.h"

template <typename Scalar>
KFMDiveConfigPtr KeyFrameMotionDive<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast<KFMDiveConfig> (this->config);
}

template <typename Scalar>
void KeyFrameMotionDive<Scalar>::initiate()
{
  //PRINT("KeyFrameMotionDive.initiate()")
  this->motionProxy->post.closeHand("LHand");
  this->motionProxy->post.closeHand("RHand");
  KeyFrameDiveTypes type = getBehaviorCast()->keyFrameDiveType;
  if (type == KeyFrameDiveTypes::IN_PLACE) {
    this->endPosture = PostureState::DIVE_IN_PLACE;
    this->runKeyFrameMotion(diveInPlace);
    this->inBehavior = true;
  } else if (type == KeyFrameDiveTypes::SUMO) {
    this->endPosture = PostureState::DIVE_SUMO;
    this->runKeyFrameMotion(diveSumo);
    this->inBehavior = true;
  } else if (type == KeyFrameDiveTypes::LEFT) {
    this->endPosture = PostureState::DIVE_LEFT;
    this->runKeyFrameMotion(diveLeft);
    this->inBehavior = true;
  } else if (type == KeyFrameDiveTypes::RIGHT) {
    this->endPosture = PostureState::DIVE_RIGHT;
    this->runKeyFrameMotion(diveRight);
    this->inBehavior = true;
  }
}

template <typename Scalar>
void KeyFrameMotionDive<Scalar>::update()
{
  //PRINT("KeyFrameMotionDive.update()")
  if (this->runTime > diveTime + this->cycleTime / 2) {
    OVAR(PostureState, MotionModule::postureState) = this->endPosture;
    finish();
  }
}

template <typename Scalar>
void KeyFrameMotionDive<Scalar>::finish()
{
  this->motionProxy->killAll();
  this->inBehavior = false;
}

template class KeyFrameMotionDive<MType>;
