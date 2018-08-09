/**
 * @file MotionModule/src/GetupModule/GetupModule.cpp
 *
 * This file implements the class GetupModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 26 Dec 2017
 */

#include "MotionModule/include/GetupModule/GetupModule.h"
#include "MotionModule/include/GetupModule/Types/KeyFrameMotionGetup.h"

boost::shared_ptr<GetupModule> GetupModule::getType(
  MotionModule* motionModule, const BehaviorConfigPtr& cfg) 
{ 
  GetupModule* gm;
  switch (cfg->type) {
      case (unsigned) MBGetupTypes::KEY_FRAME_MOTION_GETUP: 
        gm = new KeyFrameMotionGetup(motionModule, cfg); break;
      default: gm = new KeyFrameMotionGetup(motionModule, cfg); break;
  }
  return boost::shared_ptr<GetupModule>(gm);
}

