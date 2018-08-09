/**
 * @file MotionModule/include/BallThrow/BallThrow.h
 *
 * This file declares the class BallThrow
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 27 Sep 2017  
 */

#pragma once

#include "MotionModule/include/PostureModule/PostureModule.h"
#include "MotionModule/include/MotionBehavior.h"
#include "MotionModule/include/TrajectoryPlanner/TrajectoryPlanner.h"
#include "MotionModule/include/MotionConfigs/MBBallThrowConfig.h"

/** 
 * @class BallThrow
 * @brief The base class for defining a ball throwing behavior
 */
class BallThrow : public MotionBehavior
{
public:
  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   * @param name: Name of the behavior
   */
  BallThrow(
    MotionModule* motionModule,
    const BehaviorConfigPtr& config,
		const string& name = "Not assigned.") :
    MotionBehavior(motionModule, config, name),
    execTime(0.f)
  {
  }

  /**
   * Destructor
   */
  ~BallThrow()
  {
  }

  /**
   * Returns its own child based on the given type
   * 
   * @param motionModule: Pointer to base motion module
   * @param cfg: Config of the requested behavior
   * 
   * @return BehaviorConfigPtr
   */
  static boost::shared_ptr<BallThrow> getType(
    MotionModule* motionModule, const BehaviorConfigPtr& cfg);
    
  /**
   * Derived from Behavior. Child type may or may not use the same 
   * behavior config as parent.
   */
  virtual void loadExternalConfig();
protected:
  /**
	 * Returns the cast of config to MBBallThrowConfigPtr
	 */
  MBBallThrowConfigPtr getBehaviorCast();

  //! Returns true if the head is tapped once
  bool headTapCheck() {
    if (waitForHeadTap) {
      if (IVAR(vector<float>, MotionModule::touchSensors)[HEAD_TOUCH_MIDDLE] > 0.f)
        waitForHeadTap = false;
      return false;
    }
    return true;
  }

  //! Ball radius
  static float ballRadius;

  //! Time for throw motion
  float timeToThrow;

  //! Motion execution time updated after each update
  float execTime;
  
  //! Wait for a head tap to start the behavior
  bool waitForHeadTap;

  //!Dynamics Sub-Modules
  boost::shared_ptr<TrajectoryPlanner> tPlanner;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<BallThrow> BallThrowPtr;
