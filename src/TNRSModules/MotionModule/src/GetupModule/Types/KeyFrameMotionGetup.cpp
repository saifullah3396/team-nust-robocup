/**
 * @file MotionModule/GetupModule/Types/KeyFrameMotionGetup.h
 *
 * This file implements the class KeyFrameMotionGetup
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "MotionModule/include/GetupModule/Types/KeyFrameMotionGetup.h"

template <typename Scalar>
KFMGetupConfigPtr KeyFrameMotionGetup<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <KFMGetupConfig> (this->config);
}

template <typename Scalar>
void KeyFrameMotionGetup<Scalar>::initiate()
{
  LOG_INFO("KeyFrameMotionGetup.initiate()")
  this->closeHand(L_HAND);
  this->closeHand(R_HAND);
  KeyFrameGetupTypes type = this->getBehaviorCast()->keyFrameGetupType;
  if (type == KeyFrameGetupTypes::FRONT) {
    this->endPosture = PostureState::STAND;
    this->runKeyFrameMotion(getupFromFront);
    this->inBehavior = true;
  } else if (type == KeyFrameGetupTypes::BACK) {
    this->endPosture = PostureState::STAND;
    this->runKeyFrameMotion(getupFromBack);
    this->inBehavior = true;
  } else if (type == KeyFrameGetupTypes::SIT) {
    this->endPosture = PostureState::STAND;
    this->runKeyFrameMotion(getupFromSit);
    this->inBehavior = true;
  }
}

template <typename Scalar>
void KeyFrameMotionGetup<Scalar>::update()
{
  LOG_INFO("KeyFrameMotionGetup.update()")
  if (this->runTime > this->getupTime + this->cycleTime / 2) {
    OVAR(PostureState, MotionModule::postureState) = this->endPosture;
    finish();
  }
}

template <typename Scalar>
void KeyFrameMotionGetup<Scalar>::finish()
{
  this->killAllMotions();
  this->inBehavior = false;
}

template class KeyFrameMotionGetup<MType>;
