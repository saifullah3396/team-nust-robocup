/**
 * @file MotionModule/GetupModule/Types/KeyFrameMotionGetup.h
 *
 * This file declares the class KeyFrameMotionGetup
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "MotionModule/include/GetupModule/GetupModule.h"

template<typename Scalar>
class KeyFrameMotionGetup : public GetupModule<Scalar>
{
public:

  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   */
  KeyFrameMotionGetup(
    MotionModule* motionModule,
    const BehaviorConfigPtr& config) :
    GetupModule<Scalar>(motionModule, config, "KeyFrameMotionGetup"),
    getupTime(Scalar(0))
  {
  }

  /**
   * Default destructor for this class.
   */
  ~KeyFrameMotionGetup()
  {
  }
  
  /**
   * Derived from Behavior
   */ 
  void initiate();
  void update();
  void finish();
  
private:
  /**
	 * Returns the cast of config to KFMGetupConfigPtr
	 */
  KFMGetupConfigPtr getBehaviorCast();
  void setupGetupMotion(const Scalar keyFrames[][25]);
  Scalar getupTime;
};

typedef boost::shared_ptr<KeyFrameMotionGetup<MType> > KeyFrameMotionGetupPtr;
