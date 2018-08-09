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

class KeyFrameMotionGetup : public GetupModule
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
    GetupModule(motionModule, config, "KeyFrameMotionGetup"),
    getupTime(0.f)
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
  void setupGetupMotion(const float keyFrames[][25]);
  float getupTime;
};

typedef boost::shared_ptr<KeyFrameMotionGetup> KeyFrameMotionGetupPtr;
