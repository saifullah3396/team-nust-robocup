/**
 * @file MotionModule/HeadControl/Types/HeadTargetTrack.h
 *
 * This file declares the class HeadTargetTrack
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "MotionModule/include/HeadControl/HeadControl.h"

class HeadTargetTrack : public HeadControl
{
public:
  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   */
  HeadTargetTrack(
    MotionModule* motionModule,
    const BehaviorConfigPtr& config) :
    HeadControl(motionModule, config, "HeadTargetTrack"),
    targetLostTime(0.f)
  {
  }

  /**
   * Default destructor for this class.
   */
  ~HeadTargetTrack()
  {
  }
  
  /**
   * Derived from Behavior
   */ 
  void initiate();
  void update();
  void finish();
  void loadExternalConfig();
  
private:
  /**
	 * Returns the cast of config to HeadTargetTrackConfigPtr
	 */
  HeadTargetTrackConfigPtr getBehaviorCast();
  void followTarget(const Vector4f& posCam);
  
  static Vector3f pidGains;
  Vector2f intError;
  Vector2f prevCommand;
  Vector2f errorK1;
  Vector2f errorK2;
  
  HeadTargetTypes targetType; // bconfig
  float targetLostTime;
};

typedef boost::shared_ptr<HeadTargetTrack> HeadTargetTrackPtr;
