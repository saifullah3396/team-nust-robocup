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

template <typename Scalar>
class HeadTargetTrack : public HeadControl<Scalar>
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
    HeadControl<Scalar>(motionModule, config, "HeadTargetTrack"),
    targetLostTime(Scalar(0.0))
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
  void followTarget(const Matrix<Scalar, 4, 1>& posCam);
  
  static Matrix<Scalar, 3, 1> pidGains;
  Matrix<Scalar, 2, 1> intError;
  Matrix<Scalar, 2, 1> prevCommand;
  Matrix<Scalar, 2, 1> errorK1;
  Matrix<Scalar, 2, 1> errorK2;
  
  HeadTargetTypes targetType; // bconfig
  Scalar targetLostTime;
};

typedef boost::shared_ptr<HeadTargetTrack<MType> > HeadTargetTrackPtr;
