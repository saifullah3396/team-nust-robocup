/**
 * @file MotionModule/HeadControl/Types/HeadTargetSearch.h
 *
 * This file declares the class HeadTargetSearch
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "MotionModule/include/HeadControl/HeadControl.h"

class HeadTargetSearch : public HeadControl
{
public:
  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   */
  HeadTargetSearch(
    MotionModule* motionModule,
    const BehaviorConfigPtr& config) :
    HeadControl(motionModule, config, "HeadTargetSearch"),
    totalWaitTime(1.f),
    waitTime(0.f)
  {
    targetType = HeadTargetTypes::BALL;
    behaviorState = midScan;
  }

  /**
   * Default destructor for this class.
   */
  ~HeadTargetSearch()
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
	 * Returns the cast of config to HeadTargetSearchConfigPtr
	 */
  HeadTargetSearchConfigPtr getBehaviorCast();  
  bool moveHeadToTarget(const Vector4f& posCam);
  void scanEnv();
  
  float waitTime;
  float totalWaitTime; // bconfig
  HeadTargetTypes targetType; // bconfig
  
  unsigned behaviorState;
  enum behaviorState {
    midScan = 0,
    midWait,
    leftScan,
    leftWait,
    rightScan,
    rightWait,
    finishState
  };
};

typedef boost::shared_ptr<HeadTargetSearch> HeadTargetSearchPtr;
