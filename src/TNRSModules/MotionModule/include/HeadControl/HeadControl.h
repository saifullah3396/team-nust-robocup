/**
 * @file MotionModule/include/HeadControl/HeadControl.h
 *
 * This file declares the class HeadControl
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 20 Nov 2017 
 */

#pragma once

#include "MotionModule/include/MotionBehavior.h"
#include "MotionModule/include/MotionConfigs/MBHeadControlConfig.h"
#include "MotionModule/include/MotionConfigs/MBBallThrowConfig.h"
#include "MotionModule/include/KinematicsModule/LinkChain.h"
#include "MotionModule/include/KinematicsModule/TorsoState.h"
#include "MotionModule/include/KinematicsModule/Joint.h"
#include "MotionModule/include/KinematicsModule/KinematicsConsts.h"
#include "Utils/include/ConfigMacros.h"

/** 
 * @class HeadControl
 * @brief The class for controlling the robot head movement
 */
template <typename Scalar>
class HeadControl : public MotionBehavior<Scalar>
{
public:
  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   * @param name: Name of the behavior
   */
  HeadControl(
    MotionModule* motionModule,
    const BehaviorConfigPtr& config,
		const string& name = "Not assigned.") :
    MotionBehavior<Scalar>(motionModule, config, name)
  {
  }

  /**
   * Destructor
   */
  ~HeadControl()
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
  static boost::shared_ptr<HeadControl<Scalar> > getType(
    MotionModule* motionModule, const BehaviorConfigPtr& cfg);

  /**
   * Derived from Behavior. Child type may or may not use the same 
   * behavior config as parent.
   */
  virtual void loadExternalConfig() {}
  
protected:
  /**
   * Returns true if a target of type targetType is found and saves its
   * x-y-z coordinates in the input variables
   * 
   * @param targetType: Target type
   * @param targetXY: x-y coordinate output if the target is found
   * @param targetZ: z coordinate output if the target is found
   */ 
  bool findTarget(
    const HeadTargetTypes& targetType, 
    Point_<Scalar>& targetXY, 
    Scalar& targetZ) throw (BehaviorException);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<HeadControl<Scalar> > HeadControlPtr;
