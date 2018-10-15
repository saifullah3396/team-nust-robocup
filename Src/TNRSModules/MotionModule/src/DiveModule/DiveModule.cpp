/**
 * @file MotionModule/src/DiveModule/DiveModule.cpp
 *
 * This file declares the class DiveModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 26 Dec 2017
 */

#include "MotionModule/include/DiveModule/DiveModule.h"
#include "MotionModule/include/DiveModule/Types/KeyFrameMotionDive.h"

template <typename Scalar>
boost::shared_ptr<DiveModule<Scalar> > DiveModule<Scalar>::getType(
  MotionModule* motionModule, const BehaviorConfigPtr& cfg) 
{ 
  DiveModule<Scalar>* dm;
  switch (cfg->type) {
      case (unsigned) MBDiveTypes::KEY_FRAME_MOTION_DIVE: 
        dm = new KeyFrameMotionDive<Scalar>(motionModule, cfg); break;
      default: dm = new KeyFrameMotionDive<Scalar>(motionModule, cfg); break;
  }
  return boost::shared_ptr<DiveModule<Scalar> >(dm);
}

template class DiveModule<MType>;
