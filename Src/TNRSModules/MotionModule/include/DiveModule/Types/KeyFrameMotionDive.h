/**
 * @file MotionModule/DiveModule/Types/KeyFrameMotionDive.h
 *
 * This file declares the class KeyFrameMotionDive
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "MotionModule/include/DiveModule/DiveModule.h"

template <typename Scalar>
class KeyFrameMotionDive : public DiveModule<Scalar>
{
public:
  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   */
  KeyFrameMotionDive(
    MotionModule* motionModule,
    const BehaviorConfigPtr& config) :
    DiveModule<Scalar>(motionModule, config, "KeyFrameMotionDive"),
    diveTime(0.f)
  {
  }

  /**
   * Default destructor for this class.
   */
  ~KeyFrameMotionDive()
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
	 * Returns the cast of config to KFMDiveConfigPtr
	 */
  KFMDiveConfigPtr getBehaviorCast();
  void setupDive(const Scalar dive[][25]);
  Scalar diveTime;
};

typedef boost::shared_ptr<KeyFrameMotionDive<MType> > KeyFrameMotionDivePtr;
