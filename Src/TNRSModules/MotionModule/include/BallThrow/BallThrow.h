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
#include "MotionModule/include/MotionConfigs/MBBallThrowConfig.h"
#include "MotionModule/include/KinematicsModule/LinkChain.h"
#include "MotionModule/include/KinematicsModule/TorsoState.h"
#include "MotionModule/include/KinematicsModule/Joint.h"
#include "Utils/include/ConfigMacros.h"

class JointCmdsRecorder;

/** 
 * @class BallThrow
 * @brief The base class for defining a ball throwing behavior
 */
template <typename Scalar>
class BallThrow : public MotionBehavior<Scalar>
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
    MotionBehavior<Scalar>(motionModule, config, name),
    execTime(0.f),
    behaviorState(waitForHeadTap)
  {
  }

  /**
   * Destructor
   */
  ~BallThrow();

  /**
   * Returns its own child based on the given type
   * 
   * @param motionModule: Pointer to base motion module
   * @param cfg: Config of the requested behavior
   * 
   * @return BehaviorConfigPtr
   */
  static boost::shared_ptr<BallThrow<Scalar> > getType(
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

  //! waitForHeadTap state action
  void waitForHeadTapAction();

  //! Ball radius
  static Scalar ballRadius;

  //! Time for throw motion
  Scalar timeToThrow;

  //! Motion execution time updated after each update
  Scalar execTime;
  
  //! Joint recorder pointer
  JointCmdsRecorder* jcr;
  
  //! Current behavior state
  unsigned behaviorState;
  
  //! Behavior states
  enum BehaviorStates {
    waitForHeadTap,
    grabBall,
    retract,
    throwBall
  };
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
  
typedef boost::shared_ptr<BallThrow<MType> > BallThrowPtr;
