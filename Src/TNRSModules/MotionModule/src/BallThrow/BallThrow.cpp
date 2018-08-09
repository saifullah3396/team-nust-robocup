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

float BallThrow::ballRadius;

boost::shared_ptr<BallThrow> BallThrow::getType(
  MotionModule* motionModule, const BehaviorConfigPtr& cfg) 
{ 
  BallThrow* bt;
  switch (cfg->type) {
      case (unsigned) MBBallThrowTypes::WB_BALL_THROW: 
        bt = new WBBallThrow(motionModule, cfg); break;
      default: bt = new WBBallThrow(motionModule, cfg); break;
  }
  return boost::shared_ptr<BallThrow>(bt);
}

MBBallThrowConfigPtr BallThrow::getBehaviorCast()
{
  return boost::static_pointer_cast<MBBallThrowConfig> (config);
}

void BallThrow::loadExternalConfig()
{
  GET_CONFIG(
    "EnvProperties",
    (float, ballRadius, ballRadius),
  )
}
