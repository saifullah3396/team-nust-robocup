/**
 * @file MotionModule/src/MBManager.cpp
 *
 * This file implements the class MBManager
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 02 Aug 2018
 */

#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/MBManager.h"
/*#include "MotionModule/include/BalanceModule/BalanceModule.h"
#include "MotionModule/include/BallThrow/BallThrow.h"
#include "MotionModule/include/DiveModule/DiveModule.h"
#include "MotionModule/include/GetupModule/GetupModule.h"
#include "MotionModule/include/HeadControl/HeadControl.h"
#include "MotionModule/include/KickModule/KickModule.h"
#include "MotionModule/include/PostureModule/PostureModule.h"
#include "MotionModule/include/MovementModule/MovementModule.h"
*/
template <typename Scalar>
MBManager<Scalar>::MBManager(MotionModule* motionModule) :
  BehaviorManager("MBManager"),
  motionModule(motionModule)
{
}

template <typename Scalar>
bool MBManager<Scalar>::makeBehavior(
  BehaviorPtr& behavior, const BehaviorConfigPtr& cfg)
{
  /*if (cfg->baseType != BaseBehaviorType::MOTION)
    return false;
  if (cfg->id == (unsigned) MBIds::POSTURE) {
    behavior = BehaviorPtr(PostureModule<Scalar>::getType(motionModule, cfg));
  } else if (cfg->id == (unsigned) MBIds::KICK) {
    behavior = BehaviorPtr(KickModule<Scalar>::getType(motionModule, cfg));
  } else if (cfg->id == (unsigned) MBIds::BALANCE) {
    behavior = BehaviorPtr(BalanceModule<Scalar>::getType(motionModule, cfg));
  } else if (cfg->id == (unsigned) MBIds::BALL_THROW) {
    behavior = BehaviorPtr(BallThrow<Scalar>::getType(motionModule, cfg));
  } else if (cfg->id == (unsigned) MBIds::MOVEMENT) {
    behavior = BehaviorPtr(MovementModule<Scalar>::getType(motionModule, cfg));
  } else if (cfg->id == (unsigned) MBIds::HEAD_CONTROL) {
    behavior = BehaviorPtr(HeadControl<Scalar>::getType(motionModule, cfg));
  } else if (cfg->id == (unsigned) MBIds::DIVE) {
    behavior = BehaviorPtr(DiveModule<Scalar>::getType(motionModule, cfg));
  } else if (cfg->id == (unsigned) MBIds::GETUP) {
    behavior = BehaviorPtr(GetupModule<Scalar>::getType(motionModule, cfg));
  } else {
    return false;
  }*/
  return true;
}

template class MBManager<MType>;
