/**
 * @file MotionModule/include/DiveModule/DiveModule.h
 *
 * This file declares the class DiveModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 26 Dec 2017 
 */

#pragma once

#include "MotionModule/include/MotionBehavior.h"
#include "MotionModule/include/DiveModule/KeyFrameDiveTypes.h"
#include "MotionModule/include/MotionConfigs/MBDiveConfig.h"

/** 
 * @class DiveModule
 * @brief The class for generating different types of dives on the 
 *   robot.
 */
template <typename Scalar>
class DiveModule : public MotionBehavior<Scalar>
{
public:
  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   * @param name: Name of the behavior
   */
  DiveModule(
    MotionModule* motionModule,
    const BehaviorConfigPtr& config,
		const string& name = "Not assigned.") :
    MotionBehavior<Scalar>(motionModule, config, name)
  {
  }

  /**
   * Destructor
   */
  ~DiveModule()
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
  static boost::shared_ptr<DiveModule<Scalar> > getType(
    MotionModule* motionModule, const BehaviorConfigPtr& cfg);
  
  /**
   * Derived from Behavior. Child type may or may not use the same 
   * behavior config as parent.
   */
  virtual void loadExternalConfig() {}

protected:
  //! The final posture state after dive
  PostureState endPosture;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<DiveModule<MType> > DiveModulePtr;
