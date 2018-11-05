/**
 * @file MotionModule/src/BallThrow/BallThrow.cpp
 *
 * This file implements the BallThrow
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 27 Sep 2017
 */

#include "MotionModule/include/BallThrow/BallThrow.h"
#include "MotionModule/include/BallThrow/Types/WBBallThrow.h"

template <typename Scalar>
Scalar BallThrow<Scalar>::ballRadius;

template <typename Scalar>
BallThrow<Scalar>::~BallThrow()
{
}

template <typename Scalar>
boost::shared_ptr<BallThrow<Scalar> > BallThrow<Scalar>::getType(
  MotionModule* motionModule, const BehaviorConfigPtr& cfg) 
{ 
  BallThrow<Scalar>* bt;
  switch (cfg->type) {
      case (unsigned) MBBallThrowTypes::WB_BALL_THROW: 
        bt = new WBBallThrow<Scalar>(motionModule, cfg); break;
      default: bt = new WBBallThrow<Scalar>(motionModule, cfg); break;
  }
  return boost::shared_ptr<BallThrow<Scalar> >(bt);
}

template <typename Scalar>
MBBallThrowConfigPtr BallThrow<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast<MBBallThrowConfig> (this->config);
}

template <typename Scalar>
void BallThrow<Scalar>::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    GET_CONFIG(
      "EnvProperties",
      (Scalar, ballRadius, ballRadius),
    )
    loaded = true;
  }
}

template <typename Scalar>
void BallThrow<Scalar>::waitForHeadTapAction() {
  if (IVAR(vector<Scalar>, MotionModule::touchSensors)[HEAD_TOUCH_MIDDLE] > 0.f)
    behaviorState = grabBall;
}

template class BallThrow<MType>;
