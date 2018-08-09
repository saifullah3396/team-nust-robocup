/**
 * @file MotionModule/src/PostureModule/PostureModule.cpp
 *
 * This file implements the class PostureModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 13 May 2017
 */

#include "MotionModule/include/PostureModule/PostureModule.h"
#include "MotionModule/include/PostureModule/Types/InterpToPosture.h"

boost::shared_ptr<PostureModule> PostureModule::getType(
  MotionModule* motionModule, const BehaviorConfigPtr& cfg) 
{ 
  PostureModule* pm;
  switch (cfg->type) {
      case (unsigned) MBPostureTypes::INTERP_TO_POSTURE: 
        pm = new InterpToPosture(motionModule, cfg); break;
      default: pm = new InterpToPosture(motionModule, cfg); break;
  }
  return boost::shared_ptr<PostureModule>(pm);
}

MBPostureConfigPtr PostureModule::getBehaviorCast()
{
  return boost::static_pointer_cast <MBPostureConfig> (config);
}
