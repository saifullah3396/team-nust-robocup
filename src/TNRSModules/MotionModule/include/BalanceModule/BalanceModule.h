/**
 * @file MotionModule/include/BalanceModule/BalanceModule.h
 *
 * This file declares the class BalanceModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 20 April 2017
 */

#pragma once

#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/MotionBehavior.h"

/**
 * @class BalanceModule
 * @brief The class for controlling the stability of the robot.
 */
template <typename Scalar>
class BalanceModule : public MotionBehavior<Scalar>
{
public:
  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   * @param name: Name of the behavior
   */
  BalanceModule(
    MotionModule* motionModule,
    const BehaviorConfigPtr& config,
		const string& name = "Not assigned.") :
    MotionBehavior<Scalar>(motionModule, config, name)
  {
  }

  /**
   * Destructor
   */
  ~BalanceModule()
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
  static boost::shared_ptr<BalanceModule<Scalar> > getType(
    MotionModule* motionModule, const BehaviorConfigPtr& cfg);

  /**
   * Derived from Behavior. Child type may or may not use the same 
   * behavior config as parent.
   */
  virtual void loadExternalConfig() {}
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<BalanceModule<MType> > BalanceModulePtr;
