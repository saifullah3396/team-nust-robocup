/**
 * @file MotionModule/include/PostureModule/PostureModule.h
 *
 * This file declares the class PostureModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 13 May 2017 
 */

#pragma once

#include "MotionModule/include/MotionBehavior.h"
#include "MotionModule/include/MotionConfigs/MBPostureConfig.h"
#include "MotionModule/include/PostureModule/PostureDefinitions.h"
#include "Utils/include/PostureState.h"

/** 
 * @class PostureModule
 * @brief The class for sending the robot to fixed postures
 */
class PostureModule : public MotionBehavior
{
public:
  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   * @param name: Name of the behavior
   */
  PostureModule(
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
  ~PostureModule()
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
  static boost::shared_ptr<PostureModule> getType(
    MotionModule* motionModule, const BehaviorConfigPtr& cfg);

  /**
   * Derived from Behavior. Child type may or may not use the same 
   * behavior config as parent.
   */
  virtual void loadExternalConfig() {}
protected:
  /**
	 * Returns the cast of config to MBPostureConfigPtr
	 */ 
  MBPostureConfigPtr getBehaviorCast();
  
  //! Joint configuration to reach
  VectorXf jointsToReach;

  //! Time to reach posture
  float timeToReachP;

  //! Motion execution time updated after each update
  float execTime;

  //! The required target posture
  PostureState targetPosture;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<PostureModule> PostureModulePtr;
