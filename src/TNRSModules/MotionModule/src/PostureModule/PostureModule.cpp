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

template <typename Scalar>
boost::shared_ptr<PostureModule<Scalar> > PostureModule<Scalar>::getType(
  MotionModule* motionModule, const BehaviorConfigPtr& cfg) 
{ 
  PostureModule<Scalar>* pm;
  switch (cfg->type) {
      case (unsigned) MBPostureTypes::INTERP_TO_POSTURE: 
        pm = new InterpToPosture<Scalar>(motionModule, cfg); break;
      default: pm = new InterpToPosture<Scalar>(motionModule, cfg); break;
  }
  return boost::shared_ptr<PostureModule<Scalar> >(pm);
}

template <typename Scalar>
MBPostureConfigPtr PostureModule<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <MBPostureConfig> (this->config);
}

template class PostureModule<MType>;
